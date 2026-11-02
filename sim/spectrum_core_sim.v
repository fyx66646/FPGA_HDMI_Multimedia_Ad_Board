// ============================================================================
// spectrum_core_sim.v : spectrum_core 的 ModelSim 仿真副本
// 与原文件唯一差异: wire bin 的声明从输出频点段提前到 ra1 使用处之前
// (TD 综合器容忍"先用后声明", ModelSim 10.1c 严格报错)。逻辑完全等价,
// 工程 bit 仍由原文件编译, 此副本仅用于仿真。
// 由脚本生成, 源码变更后请重新生成。
// ============================================================================
// ============================================================================
// spectrum_core : 64 点基-2 FFT 音频频谱（video_clk 域，纯 FFT 直出）
// 输入 I2S 接收器的音频样本（本域信号，无跨域），左声道 24bit 取高 16bit。
// 音频链路每个样本连发两帧（样本重复法），本模块每对只取第一个 →
// 有效采样率 24kHz。每 64 个有效样本（约 2.67ms）完成一帧 FFT，
// 输出全部 32 个频点（1..32 = 375Hz..12kHz，全部落在声音有效频段）
// 的能量等级到 O_levels。不做任何后处理：无平滑、无补偿、无包络。
//
// 精度：频率分辨率 375Hz；电平 6bit（0..63），对数刻度 4 细分级/倍频程
// （msb 位置*4 + 次高 2 位），幅度 ≈ max(|re|,|im|) + min/2。
//
// 架构：串行时域抽取（DIT）FFT，每拍一个蝶形，6 级 x 32 蝶 = 192 拍，
// 再加 32 拍输出 → 全程约 230 拍（<< 一块的 66560 拍），单块 RAM 即可，
// FFT 期间丢 ≤1 个样本，对可视化无感。
// 蝶形：输入 >>1 定标防溢出，W 为 Q14 旋转因子（case 查表），乘法 4 个/拍。
// ============================================================================
module spectrum_core (
    input  wire        I_clk,          // video_clk 25MHz
    input  wire        I_rst,          // 高有效

    input  wire        I_sample_valid, // 音频样本有效（48kHz）
    input  wire [23:0] I_sample_left,  // 左声道 24bit PCM

    output reg  [191:0] O_levels       // 32 频段 x 6bit 电平（0..63），每块更新
);

    localparam [1:0] FFT_IDLE = 2'd0;
    localparam [1:0] FFT_RUN  = 2'd1;
    localparam [1:0] FFT_OUT  = 2'd2;

    // 64 字 x 32bit {im, re} 采样/变换缓冲
    // 2026-10-26:双份复制(消 PHY-8023)曾致布线全局重排、疑似引发 SD 卡顿回归,
    // 暂回退单份;若再遇 PHY-8023 用 eco 重跑或重新评估
    reg [31:0] mem [0:63];

    reg [5:0]  waddr;    // 采样写入地址 0..63
    reg [1:0]  fstate;   // FFT 状态机
    reg [7:0]  fcnt;     // FFT_RUN: 蝶形计数 0..191；FFT_OUT: 频点计数 0..31
    reg        samp_tgl; // 重复样本对分频：每个样本对只取第一个

    wire signed [15:0] sample = I_sample_left[23:8];

    // ---------------- 旋转因子 W_k = exp(-j2πk/64)，Q14 ----------------
    function [31:0] tw_lut;   // {sin, cos}
        input [4:0] k;
        reg [15:0] c, s;
        begin
            case (k)
                5'd0:  begin c = 16'sd16384; s = 16'sd0; end
                5'd1:  begin c = 16'sd16305; s = -16'sd1606; end
                5'd2:  begin c = 16'sd16069; s = -16'sd3196; end
                5'd3:  begin c = 16'sd15679; s = -16'sd4756; end
                5'd4:  begin c = 16'sd15137; s = -16'sd6270; end
                5'd5:  begin c = 16'sd14449; s = -16'sd7723; end
                5'd6:  begin c = 16'sd13623; s = -16'sd9102; end
                5'd7:  begin c = 16'sd12665; s = -16'sd10394; end
                5'd8:  begin c = 16'sd11585; s = -16'sd11585; end
                5'd9:  begin c = 16'sd10394; s = -16'sd12665; end
                5'd10: begin c = 16'sd9102;  s = -16'sd13623; end
                5'd11: begin c = 16'sd7723;  s = -16'sd14449; end
                5'd12: begin c = 16'sd6270;  s = -16'sd15137; end
                5'd13: begin c = 16'sd4756;  s = -16'sd15679; end
                5'd14: begin c = 16'sd3196;  s = -16'sd16069; end
                5'd15: begin c = 16'sd1606;  s = -16'sd16305; end
                5'd16: begin c = 16'sd0;     s = -16'sd16384; end
                5'd17: begin c = -16'sd1606; s = -16'sd16305; end
                5'd18: begin c = -16'sd3196; s = -16'sd16069; end
                5'd19: begin c = -16'sd4756; s = -16'sd15679; end
                5'd20: begin c = -16'sd6270; s = -16'sd15137; end
                5'd21: begin c = -16'sd7723; s = -16'sd14449; end
                5'd22: begin c = -16'sd9102; s = -16'sd13623; end
                5'd23: begin c = -16'sd10394; s = -16'sd12665; end
                5'd24: begin c = -16'sd11585; s = -16'sd11585; end
                5'd25: begin c = -16'sd12665; s = -16'sd10394; end
                5'd26: begin c = -16'sd13623; s = -16'sd9102; end
                5'd27: begin c = -16'sd14449; s = -16'sd7723; end
                5'd28: begin c = -16'sd15137; s = -16'sd6270; end
                5'd29: begin c = -16'sd15679; s = -16'sd4756; end
                5'd30: begin c = -16'sd16069; s = -16'sd3196; end
                default: begin c = -16'sd16305; s = -16'sd1606; end
            endcase
            tw_lut = {s, c};
        end
    endfunction

    // ---------------- 输出频点：全部 1..32 ----------------
    function [5:0] bin_lut;
        input [4:0] i;
        begin
            bin_lut = {1'b0, i} + 6'd1;
        end
    endfunction

    // ---------------- 蝶形寻址（组合，来自 fcnt） ----------------
    wire [2:0] stage  = fcnt[7:5];                     // 0..5
    wire [4:0] bcount = fcnt[4:0];                     // 0..31
    wire [5:0] smask  = (6'd1 << stage) - 6'd1;        // (1<<stage)-1
    wire [4:0] pos    = bcount & smask[4:0];
    wire [4:0] group  = bcount >> stage;
    wire [6:0] addr1  = ({2'b0, group} << (stage + 3'd1)) | {2'b0, pos};
    wire [6:0] addr2  = addr1 | (7'd1 << stage);
    wire [4:0] twidx  = pos << (3'd5 - stage);         // 0..31

    // ---------------- 蝶形运算（组合，输入为同步读锁存值） ----------------
    // 同步读流水：每拍锁存 mem 读出与地址/旋转因子，消除组合三读的多路选择结构
    // （组合多读推断的分布式 RAM 曾在布局阶段产生网络位置冲突）。
    // 加固(2026-10-23)：M 读口与 A 读口复用——FFT_OUT 阶段只读 mem[bin]，
    // 与蝶形阶段读 addr1 时间互斥，用 fstate 切换读地址即可；
    // RUN 末拍冲刷读 bin 保持 OUT 首拍 rdA=mem[bin(0)] 与原 rdM 流水一致。
    // RAM 端口由 3读+2写 降为 2读+2写，消除残留的 PHY-8023 概率性布线冲突。
    reg [6:0]  a1_q, a2_q;
    reg [4:0]  tw_q;
    reg [5:0]  bin_q;
    reg [6:0]  ra1_r, ra2_r;
    reg [6:0]  a1_d, a2_d;
    reg [4:0]  twidx_d;
    reg [5:0]  bin_d;
    reg [31:0] rdA, rdB;

    // 读口 A 地址：蝶形阶段读 addr1，OUT/冲刷拍读 bin（M/A 读口复用）
    // 2026-10-26 结构级修复：地址打一拍寄存后再驱动 RAM 读口——
    // 原组合复用网(mux/XOR/AND-OR 各形态)反复触发 PHY-8023 局部布线冲突，
    // 寄存器化后 RAM 读口由寄存器直驱，消除冲突结构。
    // 代价：数据流水整体后移一拍（a1_d 等控制流水同步加一级），
    // FFT_RUN 冲刷拍 192→193，行为逐拍等价。
    wire       flush_cyc = (fstate == FFT_RUN) && (fcnt == 8'd193);
    wire       out_phase = (fstate == FFT_OUT) || flush_cyc;
    wire [5:0]  bin = bin_lut(fcnt[4:0]);   // 仿真副本: 声明提前到使用处之前(功能等价)
    wire [6:0] ra1_diff  = addr1 ^ bin;                    // 17:32 验证可布线的 XOR 形态
    wire [6:0] ra1       = addr1 ^ (out_phase ? ra1_diff : 7'd0);

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            a1_q    <= 7'd0;
            a2_q    <= 7'd0;
            tw_q    <= 5'd0;
            bin_q   <= 6'd0;
            ra1_r   <= 7'd0;
            ra2_r   <= 7'd0;
            a1_d    <= 7'd0;
            a2_d    <= 7'd0;
            twidx_d <= 5'd0;
            bin_d   <= 6'd0;
            rdA     <= 32'd0;
            rdB     <= 32'd0;
        end
        else begin
            // 地址先寄存，RAM 读口由寄存器直驱（不再有组合复用网进 RAM）
            ra1_r   <= ra1;
            ra2_r   <= addr2;
            rdA     <= mem[ra1_r];
            rdB     <= mem[ra2_r];
            // 控制流水同步加一级，与数据保持对齐
            a1_q    <= addr1;
            a2_q    <= addr2;
            tw_q    <= twidx;
            bin_q   <= bin;
            a1_d    <= a1_q;
            a2_d    <= a2_q;
            twidx_d <= tw_q;
            bin_d   <= bin_q;
        end
    end

    wire [31:0] dA = rdA;
    wire [31:0] dB = rdB;

    wire signed [15:0] Ar = $signed(dA[15:0])  >>> 1;
    wire signed [15:0] Ai = $signed(dA[31:16]) >>> 1;
    wire signed [15:0] Br = $signed(dB[15:0])  >>> 1;
    wire signed [15:0] Bi = $signed(dB[31:16]) >>> 1;

    wire [31:0] TW = tw_lut(twidx_d);
    wire signed [15:0] Wr = $signed(TW[15:0]);        // cos
    wire signed [15:0] Wi = $signed(TW[31:16]);       // sin

    wire signed [31:0] p1 = Br * Wr;
    wire signed [31:0] p2 = Bi * Wi;
    wire signed [31:0] p3 = Br * Wi;
    wire signed [31:0] p4 = Bi * Wr;
    wire signed [17:0] t_re = (p1 - p2) >>> 14;
    wire signed [17:0] t_im = (p3 + p4) >>> 14;

    wire signed [17:0] n1r = Ar + t_re;
    wire signed [17:0] n1i = Ai + t_im;
    wire signed [17:0] n2r = Ar - t_re;
    wire signed [17:0] n2i = Ai - t_im;

    wire [31:0] new1 = {n1i[15:0], n1r[15:0]};
    wire [31:0] new2 = {n2i[15:0], n2r[15:0]};

    // ---------------- 输出频点幅度 → 6bit 高精度电平（组合，基于同步读锁存值） ----------------
    wire [31:0] dM  = rdA;      // OUT 阶段 A 读口已在读 mem[bin]，复用 rdA 即原 rdM
    wire signed [15:0] Mre = $signed(dM[15:0]);
    wire signed [15:0] Mim = $signed(dM[31:16]);
    wire [15:0] abs_r = Mre[15] ? (~Mre[15:0] + 16'd1) : Mre[15:0];
    wire [15:0] abs_i = Mim[15] ? (~Mim[15:0] + 16'd1) : Mim[15:0];
    wire [15:0] mx = (abs_r > abs_i) ? abs_r : abs_i;
    wire [15:0] mn = (abs_r > abs_i) ? abs_i : abs_r;
    wire [16:0] mag = {1'b0, mx} + {1'b0, mn[15:1]};

    reg [4:0] msb_pos;
    reg       found;
    integer   bi;
    always @(*) begin
        msb_pos = 5'd0;
        found   = 1'b0;
        for (bi = 16; bi >= 0; bi = bi - 1) begin
            if (!found && mag[bi]) begin
                found   = 1'b1;
                msb_pos = bi;
            end
        end
    end

    // 对数刻度 4 细分级/倍频程：level = msb*4 + 次高2位 - 16，限幅 0..63
    wire [16:0] mag_shifted = (mag >> (msb_pos - 5'd2));   // msb_pos>=2 时有效
    wire [1:0]  frac2 = (msb_pos >= 5'd2) ? mag_shifted[1:0] : 2'b00;
    wire [6:0]  lv_raw = {2'b0, msb_pos} * 3'd4 + {5'b0, frac2};   // 0..67
    wire signed [7:0] lv_tmp = {1'b0, lv_raw} - 8'd16;
    wire [5:0]  lv_out6 = lv_tmp[7] ? 6'd0 : ((lv_tmp > 8'd63) ? 6'd63 : lv_tmp[5:0]);

    // 槽位用锁存的 bin_d（与 rdA 同拍对齐）
    wire [7:0]   shamt   = {2'b0, bin_d} * 3'd6;          // 0..186
    wire [191:0] lv_mask  = 192'h00000000000000000000000000000000000000000000003F << shamt;
    wire [191:0] lv_shift = {186'b0, lv_out6} << shamt;

    // ---------------- 主状态机 ----------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            waddr    <= 6'd0;
            fcnt     <= 8'd0;
            fstate   <= FFT_IDLE;
            samp_tgl <= 1'b0;
            O_levels <= 192'd0;
        end
        else begin
            case (fstate)
                FFT_IDLE: begin
                    if (I_sample_valid) begin
                        // 音频链路每个样本连发两帧（样本重复法），这里每对只取
                        // 第一个 → 有效采样率 24kHz，64 点 FFT 分辨率 375Hz，
                        // 32 个频点 = 375Hz..12kHz，全部落在声音有效频段
                        samp_tgl <= ~samp_tgl;
                        if (~samp_tgl) begin
                            // 修复1: 蝶形引擎要求输入按位反序(自然序入会得弥散谱),
                            // 采样写入地址做 6 位位反序
                            mem[{waddr[0], waddr[1], waddr[2],
                                 waddr[3], waddr[4], waddr[5]}] <= {16'h0000, sample};
                            if (waddr == 6'd63) begin
                                waddr  <= 6'd0;
                                fcnt   <= 8'd0;
                                fstate <= FFT_RUN;
                            end
                            else begin
                                waddr <= waddr + 6'd1;
                            end
                        end
                    end
                end

                FFT_RUN: begin
                    // 修复2: 流水预热期(fcnt<2)读地址为陈旧值, 跳过写回,
                    // 避免 (0,1) 蝶形被重复处理
                    if (fcnt >= 8'd2) begin
                        mem[a1_d] <= new1;      // 同步读流水：写回上一拍锁存的地址
                        mem[a2_d] <= new2;
                    end
                    if (fcnt == 8'd193) begin    // 192 蝶形 + 2 拍流水冲刷
                        fcnt   <= 8'd0;
                        fstate <= FFT_OUT;
                    end
                    else begin
                        fcnt <= fcnt + 8'd1;
                    end
                end

                FFT_OUT: begin
                    // 纯 FFT 直出：逐频点原始电平，不做任何平滑
                    O_levels <= (O_levels & ~lv_mask) | lv_shift;
                    if (fcnt == 8'd32) begin      // 32 频点 + 1 拍流水冲刷
                        fcnt   <= 8'd0;
                        fstate <= FFT_IDLE;
                    end
                    else begin
                        fcnt <= fcnt + 8'd1;
                    end
                end

                default: begin
                    waddr  <= 6'd0;
                    fcnt   <= 8'd0;
                    fstate <= FFT_IDLE;
                end
            endcase
        end
    end

endmodule
