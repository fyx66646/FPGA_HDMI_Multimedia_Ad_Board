// ============================================================================
// sd_wav_i2s_player : TF 卡音频 → I2S 发送（64fs 时序，带音量/倍速）
// 时序与原 hdmi_audio_tone_i2s_64fs 完全一致，下游 I2S_receiver 无需改动：
//   MCLK = 12.288MHz，BCLK = MCLK/2 = 6.144MHz，LRCK = BCLK/128 = 48kHz
//   每声道 64bit 时隙，24bit 样本 MSB 对齐放在 [63:40]
//
// 【实测校准 2026-10-21】1.0x 正常速率 = 每 LRCK 周期（左右槽）消费一对样本。
// 倍速：acc 于每周期（右槽结束拍）累加 rate(8.8 定点，128..512 = 0.5x..2.0x)。
//   右槽结束拍：装左槽 = cur.左；acc+rate ≥ 512 且 nxt 就绪 → 换对（2.0x 第一对）
//   左槽结束拍：装右槽 = cur.右（读换对后的老值，2.0x 时与左槽跨对）；acc ≥ 256 换对
//   1.0x：右槽结束拍不换、左槽结束拍换 → 每周期 1 对，左右同对
//   2.0x：两拍各换一次 → 每周期 2 对，左右槽跨对（错位约 10us，不可闻）
//   0.5x：每 2 周期换 1 次 → 每对播 2 周期
//   欠载（nxt 未备好）不换对：重复当前对，FIFO 恢复后自动追上（acc 饱和防溢出）。
//
// 样本通路（16bit PCM，左右成对）：
//   audio_fifo → half(暂存左) → nxt(预装下一对) → cur(正在播的一对)
// 音量：装载移位寄存器时对样本乘 I_vol_gain(0..256，>>8)。
// I_vol_gain / I_rate 来自 video_clk 域（准静态，2 拍同步后使用）。
// ============================================================================
module sd_wav_i2s_player (
    input  wire        I_mclk,         // 12.288MHz
    input  wire        I_rst,          // 高有效

    // 音频 FIFO 读侧（I_mclk 域）
    input  wire        I_fifo_empty,
    output reg         O_fifo_re,
    input  wire [15:0] I_fifo_dout,
    input  wire        I_fifo_valid,

    // 音量/倍速（video_clk 域准静态参数，内部 2 拍同步）
    input  wire [9:0]  I_vol_gain,     // 0..256 = 0%..100%
    input  wire [9:0]  I_rate,         // 8.8 定点 128..512 = 0.5x..2.0x

    // I2S 输出
    output reg         O_i2s_BCLK,
    output reg         O_i2s_LRCK,
    output reg         O_i2s_DOUT
);

    // ---------------- 参数 2 拍同步（video → mclk，准静态） ----------------
    reg [9:0] gain_s0, gain_s1;
    reg [9:0] rate_s0, rate_s1;

    always @(posedge I_mclk or posedge I_rst) begin
        if (I_rst) begin
            gain_s0 <= 10'd256;    // 上电 100% 音量
            gain_s1 <= 10'd256;
            rate_s0 <= 10'd256;    // 上电 1.0x
            rate_s1 <= 10'd256;
        end
        else begin
            gain_s0 <= I_vol_gain; gain_s1 <= gain_s0;
            rate_s0 <= I_rate;     rate_s1 <= rate_s0;
        end
    end

    // ---------------- 样本对缓冲 ----------------
    reg        cv;             // 当前对有效
    reg [15:0] cl, cr;         // 当前对：左/右
    reg        nv;             // 下一对有效
    reg [15:0] nl, nr;         // 下一对：左/右
    reg        half_v;         // 半对暂存（左样本等待配右）
    reg [15:0] half;

    // 倍速累加器：acc ∈ [0,512)，每周期（右槽结束拍）+rate
    reg [10:0] acc;

    reg [63:0] shift_reg;
    reg [5:0]  bit_cnt;

    // ---------------- 音量乘法（装载到移位寄存器前的组合缩放） ----------------
    wire signed [31:0] l_cur_sc  = ($signed(cl)) * $signed({1'b0, gain_s1});
    wire signed [31:0] l_next_sc = ($signed(nl)) * $signed({1'b0, gain_s1});
    wire signed [31:0] r_sc      = ($signed(cr)) * $signed({1'b0, gain_s1});
    wire [15:0] l_cur  = l_cur_sc  >>> 8;
    wire [15:0] l_next = l_next_sc >>> 8;
    wire [15:0] r_out  = r_sc      >>> 8;

    // 槽边界换对决策（组合，读老值）
    wire [10:0] acc_next_r = acc + {1'b0, rate_s1};        // 右槽结束拍累加后
    wire        swap_r     = (acc_next_r >= 11'd512) && nv; // 2.0x：周期首对
    wire        swap_l     = (acc >= 11'd256) && nv;        // 1.0x/0.5x：周期末对

    always @(posedge I_mclk or posedge I_rst) begin
        if (I_rst) begin
            O_fifo_re   <= 1'b0;
            cv          <= 1'b0;
            cl          <= 16'd0;
            cr          <= 16'd0;
            nv          <= 1'b0;
            nl          <= 16'd0;
            nr          <= 16'd0;
            half_v      <= 1'b0;
            half        <= 16'd0;
            acc         <= 11'd0;
            shift_reg   <= 64'd0;
            bit_cnt     <= 6'd0;
            O_i2s_BCLK  <= 1'b0;
            O_i2s_LRCK  <= 1'b0;
            O_i2s_DOUT  <= 1'b0;
        end
        else begin
            // ---------- FIFO 弹数：凑对进 nxt ----------
            O_fifo_re <= 1'b0;
            if (I_fifo_valid) begin
                if (!half_v) begin
                    half   <= I_fifo_dout;      // 先收左
                    half_v <= 1'b1;
                end
                else if (!nv) begin
                    nl     <= half;             // 凑成一对
                    nr     <= I_fifo_dout;
                    nv     <= 1'b1;
                    half_v <= 1'b0;
                end
            end
            else if (((!half_v) || (half_v && !nv)) && !I_fifo_empty)
                O_fifo_re <= 1'b1;

            // ---------- I2S 位引擎（与原 tone 模块逐拍一致） ----------
            O_i2s_BCLK <= ~O_i2s_BCLK;

            if (O_i2s_BCLK == 1'b1) begin
                O_i2s_DOUT <= shift_reg[63];
                shift_reg  <= {shift_reg[62:0], 1'b0};

                if (bit_cnt == 6'd63) begin
                    bit_cnt <= 6'd0;
                    if (O_i2s_LRCK == 1'b0) begin
                        // 左槽结束 → 装右槽（读换对后的老值：1.0x 同对、2.0x 跨对）
                        O_i2s_LRCK <= 1'b1;
                        if (cv)
                            shift_reg <= {r_out, 8'h00, 40'd0};
                        else
                            shift_reg <= 64'd0;             // 欠载静音
                        // 周期末换对点（1.0x/0.5x 在此换；2.0x 的第二对）
                        if (swap_l) begin
                            cl <= nl;
                            cr <= nr;
                            cv <= 1'b1;
                            nv <= 1'b0;                      // 释放 nxt 让 FIFO 续装
                            acc <= acc - 11'd256;
                        end
                    end
                    else begin
                        // 右槽结束 = 周期边界：装左槽、累加倍速、必要时换周期首对
                        O_i2s_LRCK <= 1'b0;
                        acc <= swap_r ? (acc_next_r - 11'd256)
                                      : ((acc_next_r > 11'd511) ? 11'd511
                                                                : acc_next_r);
                        if (cv)
                            shift_reg <= {l_cur, 8'h00, 40'd0};
                        else
                            shift_reg <= 64'd0;             // 欠载静音
                        if (swap_r) begin
                            cl <= nl;
                            cr <= nr;
                            cv <= 1'b1;
                            nv <= 1'b0;
                        end
                    end
                end
                else begin
                    bit_cnt <= bit_cnt + 6'd1;
                end
            end
        end
    end

endmodule
