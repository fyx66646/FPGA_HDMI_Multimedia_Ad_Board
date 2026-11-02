// ============================================================================
// snow_overlay : 粒子特效（video_clk 域，BRAM 行缓冲架构）—— 雪花 / 星空
// I_lvl：0=无 2=雪花(64 片下落) 3=星空(40 颗四芒星闪烁)
//
// 雪花：64 片三种档位（视差层次）：
//   cls0 远：1x1 像素 0xFF，速度 1px/帧，摆幅 4
//   cls1 中：2x2 像素 0xE8，速度 2px/帧，摆幅 7
//   cls2 近：3x3 像素 0xD8，速度 3px/帧，摆幅 10
//
// 星空：40 颗四芒星（十字形，|dx|,|dy|<=2 且 dx==0 或 dy==0），位置固定
//   （star_xy 常量表，Python 最小间距 60px 撒点生成，_star_gen.py）。
//   每星 6 位闪烁相位按帧 +1/2/3（各自周期 0.7~2.1s），相位高 2 位分四段：
//   亮(中心FF/内臂D0/外臂B8) → 微(D0/B8/B8) → 灭 → 微 —— 交替闪烁出没。
//
// 架构（2026-10-24 第二版，替代 64 路并行比较——那版 4.8K LUT 触发
// PHY-9009 M-Slice 超限）：串行化 + 行缓冲。
//   flake_mem(BRAM 64x35)：雪花状态 {xb,y,cls,swp,tdir,lfsr}
//   star_ph/star_st(寄存器 40x6/40x2)：星空相位/步长（位置为常量）
//   line_mem (BRAM 1280x4)：两行乒乓线缓冲（值 0=无 1..5=灰度索引），
//     真双口自毁式读取：显示端口读出条目同时写 0，条目在被其所属行
//     显示的瞬间自我清除——乒乓两行后回来必然干净（免清零免标签；
//     行号标签防不了上一帧残影=拖尾，帧号标签丢行身份，均不可用）
//   每帧时序：vs 上升沿后的消隐期先跑雪花"状态更新遍"(64 片 x 2 拍，
//     星空相位于 vs 沿并行 +1 无需遍)再填第 0 行；
//     活动行 r 显示期间顺序遍历粒子(雪花 64 片/星空 40 星，每粒 8 拍)
//     填第 r+1 行的命中像素。一套共享数据通路，O(1) 逻辑量。
//
// 雪花每片数学与 doc/convert/_snow_preview.py 逐位同构；星空亮度分级
// 中心0xFF/内臂0xD0/外臂0xB8（g_val 表：1=FF 2=E8 3=D8 4=B8 5=D0）。
//
// 层级：链路最上游（vout_data 之后、viz_overlay 之前）——粒子在频谱条/
// 字幕/转场特效之下，从它们"后面"经过；OSD 仍在最上。
// 像素通路 1 拍流水（BRAM 读延迟），de/vs/rgb 同拍顺延，下游无感。
// ============================================================================
module snow_overlay (
    input  wire        I_clk,          // video_clk 25MHz
    input  wire        I_rst,          // 高有效

    input  wire        I_vs,
    input  wire        I_de,
    input  wire [23:0] I_rgb,
    input  wire [1:0]  I_lvl,          // 0=无 2=雪花(64片) 3=星空(40星)

    output wire        O_vs,
    output wire        O_de,
    output wire [23:0] O_rgb
);

    wire snow_on  = (I_lvl == 2'd2);   // 雪花模式（状态更新遍/初始化遍使能）
    wire star_on  = (I_lvl == 2'd3);   // 星空模式
    wire any_on   = (I_lvl != 2'd0);   // 填充/显示使能

    // ---------------- 显示计数（与上游 I_de/I_rgb 同拍） ----------------
    reg        de_d, vs_d;
    reg [10:0] x_cnt, y_cnt;
    reg        in_blank;                  // vs 上升沿→第一列 de 前：跑更新遍+填第 0 行

    wire vs_rise = I_vs & ~vs_d;
    wire de_rise = I_de & ~de_d;

    // ---------------- 行内相位计数 0..799（vs/de 上升沿复位） ----------------
    reg [9:0] ph;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            de_d     <= 1'b0;
            vs_d     <= 1'b0;
            x_cnt    <= 11'd0;
            y_cnt    <= 11'd0;
            in_blank <= 1'b1;
            ph       <= 10'd0;
        end
        else begin
            de_d <= I_de;
            vs_d <= I_vs;

            if (I_de)
                x_cnt <= (x_cnt == 11'd639) ? 11'd0 : (x_cnt + 11'd1);
            else
                x_cnt <= 11'd0;

            if (vs_rise) begin
                y_cnt    <= 11'd0;
                in_blank <= 1'b1;
            end
            else if (de_rise) begin
                in_blank <= 1'b0;                    // 第一列 de：消隐期结束
                if (!in_blank && (y_cnt != 11'd479))
                    y_cnt <= y_cnt + 11'd1;          // 后续每行 de 上升沿推进行号
            end

            if (vs_rise || de_rise)
                ph <= 10'd0;
            else if (ph < 10'd799)
                ph <= ph + 10'd1;
        end
    end

    // ---------------- 雪花状态 BRAM（64 x 35，简单双口） ----------------
    // {xb[34:25], y[24:16], cls[15:14], swp[13:9], tdir[8], lfsr[7:0]}
    (* ram_style = "bram" *) reg [34:0] flake_mem [0:63];
    reg [34:0] frd;
    wire       fwe;
    wire [5:0] fwa;
    wire [34:0] fwd;
    wire [5:0] fra_r;

    always @(posedge I_clk) begin
        if (fwe)
            flake_mem[fwa] <= fwd;
        frd <= flake_mem[fra_r];
    end

    // ---------------- 线缓冲 BRAM（1280 x 4，两行乒乓，真双口） ----------------
    // 地址 = {行奇偶, x}，值 0=无 1/2/3=cls+1。
    // 自毁式读取（2026-10-24 定稿）：显示端口读出条目的同时写 0——每个条目
    // 在被其所属行显示的瞬间自我清除，乒乓两行后回来必然干净，免清零也免标签。
    // （此前两版教训：行号标签只防两行前乒乓残影、防不了上一帧残影，雪片慢速
    //   漂移的轨迹会逐帧积累成拖尾；帧号标签则丢行身份，同帧同行bank全亮。）
    // 口A=显示读+自毁清（I_de 期间），口B=填充写——活动期两口恒写不同 bank。
    (* ram_style = "bram" *) reg [3:0] line_mem [0:1279];
    reg [3:0] lrd;
    wire      lwe;
    wire [10:0] lwa;
    wire [3:0] lwd;

    wire [10:0] lra = {y_cnt[0], x_cnt[9:0]};

    always @(posedge I_clk) begin
        lrd <= line_mem[lra];
        if (I_de)
            line_mem[lra] <= 4'd0;      // 读出自毁（read-first）
        if (lwe)
            line_mem[lwa] <= lwd;       // 填充写（另一 bank）
    end

    // ---------------- 初始化 / 更新 / 填充 序列 ----------------
    reg init_done;                       // 雪花状态已初始化（首次消隐期完成）
    reg ready;                           // 首行已填充，可参与融合

    // 目标行：消隐期填第 0 行；活动行 r 填第 r+1 行
    wire [10:0] row_fill = in_blank ? 11'd0 : (y_cnt + 11'd1);
    wire        fill_row_valid = in_blank || (y_cnt < 11'd479);

    // 更新遍只在消隐期跑；初始化遍替代首次更新遍
    wire in_init  = in_blank && !init_done;
    wire in_upd   = in_blank &&  init_done;

    // 填充遍相位基址：消隐期在初始化/更新遍之后
    wire [9:0] fill_base = in_blank ? (init_done ? 10'd128 : 10'd64) : 10'd0;
    wire [9:0] q = ph - fill_base;                       // 填充遍内相位
    wire       fill_active = any_on && fill_row_valid && (ph >= fill_base) &&
                             (q < 10'd512);
    wire [5:0] fi   = q[8:3];                            // 填充遍粒子序号
    wire [2:0] fsub = q[2:0];                            // 粒子内子相位

    wire [6:0] n_act = {5'd0, I_lvl} * 7'd32;            // 雪花 0/64(档2→64)

    // 初始化值（由序号组合生成）
    function [34:0] flake_init;
        input [5:0] i;
        reg [9:0] xb0;
        reg [8:0] y0;
        begin
            xb0 = {3'd0, i, 3'b000} + {4'd0, i, 1'b0};   // i*10 = 0..630
            y0  = {3'd0, i, 3'b000} - {3'd0, i};         // i*7  = 0..441
            flake_init = {xb0, y0,
                          (i[1:0] == 2'd3) ? 2'd0 : i[1:0],
                          {2'b0, i[2:0]}, i[0],
                          8'hA5 ^ {2'b00, i}};
        end
    endfunction

    // ---------------- 从 frd 拆字段 ----------------
    wire [9:0] s_xb   = frd[34:25];
    wire [8:0] s_y    = frd[24:16];
    wire [1:0] s_cls  = frd[15:14];
    wire [4:0] s_swp  = frd[13:9];
    wire       s_tdir = frd[8];
    wire [7:0] s_lfsr = frd[7:0];

    function [7:0] lfsr_step;
        input [7:0] v;
        begin
            lfsr_step = (v >> 1) ^ (v[0] ? 8'hB8 : 8'h00);
        end
    endfunction

    wire [7:0] v_n   = lfsr_step(s_lfsr);
    wire [2:0] spd   = {1'b0, s_cls} + 3'd1;             // 1/2/3
    wire [4:0] amp   = 4'd4 + {3'd0, s_cls} * 3'd3;      // 4/7/10
    wire [4:0] tmax  = {amp, 1'b0};                      // 8/14/20

    // ---------------- 更新遍（消隐期 ph 0..127，偶拍读、奇拍写回） ----------------
    wire       upd_rd = in_upd && snow_on && (ph < 10'd128) && (ph[0] == 1'b0);
    wire [5:0] ui     = ph[6:1];
    wire       upd_wr = in_upd && snow_on && (ph < 10'd128) && (ph[0] == 1'b1)
                        && (ui < n_act);

    wire [9:0]  y_n10 = {1'b0, s_y} + {7'd0, spd};
    wire        rsp   = (y_n10 >= 10'd480);
    wire [11:0] xr5   = {4'd0, v_n} + {1'b0, v_n, 2'b00};       // v*5
    wire [9:0]  xb_rsp = xr5[10:1];                             // (v*5)>>1
    wire [4:0]  swp_n = s_tdir ? (s_swp + 5'd1) : (s_swp - 5'd1);
    wire        bounce = (swp_n == 5'd0) || (swp_n >= tmax);
    reg  [9:0]  xb_n;
    always @(*) begin
        if (rsp)
            xb_n = xb_rsp;
        else if (v_n[1:0] == 2'b00)
            xb_n = (s_xb == 10'd0) ? 10'd639 : (s_xb - 10'd1);
        else if (v_n[1:0] == 2'b11)
            xb_n = (s_xb == 10'd639) ? 10'd0 : (s_xb + 10'd1);
        else
            xb_n = s_xb;
    end

    wire [34:0] st_next = {xb_n, rsp ? (y_n10[8:0] - 9'd480) : y_n10[8:0],
                           s_cls, swp_n, bounce ? ~s_tdir : s_tdir, v_n};

    // ---------------- 填充遍（每片 8 拍：0 读 / 1 算 / 2..4 写 / 5..7 闲） ----------------
    // （读地址在 fra_r 复用，fsub==1 时 frd 有效才参与计算）

    // 目标行 y 窗口（cls0 [0,0]/cls1 [-1,0]/cls2 [-1,+1]）
    wire signed [10:0] dy = $signed({2'b00, s_y}) - $signed({1'b0, row_fill});
    wire y_win = (s_cls == 2'd0) ? (dy == 11'sd0) :
                 (s_cls == 2'd1) ? ((dy >= -11'sd1) && (dy <= 11'sd0)) :
                                   ((dy >= -11'sd1) && (dy <= 11'sd1));

    // 有效 x = (xb + swp + 640 - amp) mod 640，范围 [630,1289] 三段折叠
    wire [10:0] acc = {1'b0, s_xb} + {6'd0, s_swp} + 11'd640 - {7'd0, amp};
    wire [9:0]  fx  = (acc >= 11'd1280) ? (acc - 11'd1280) :
                      (acc >= 11'd640)  ? (acc - 11'd640)  : acc[9:0];

    reg        hit_r;
    reg [9:0]  fx_r;
    reg [1:0]  cls_r;
    // fsub==1 装载后保持到下一片——写拍(2/3/4)都用同一片的命中结果。
    // 若每拍默认清零，只有第一写拍有效：cls0 完全不可见、cls1/2 只剩
    // 左缘一列，配上 2~3 行 y 窗就成了每片雪后面的细竖"拖尾"。
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            hit_r <= 1'b0;
            fx_r  <= 10'd0;
            cls_r <= 2'd0;
        end
        else if (fill_active && snow_on && (fsub == 3'd1)) begin
            hit_r <= y_win;
            fx_r  <= fx;
            cls_r <= s_cls;
        end
    end

    // ---------------- 星空：40 颗四芒星（位置常量 + 相位寄存器） ----------------
    // 位置由 _star_gen.py 最小间距 60px 撒点生成（边缘留白 12px，±2 无需回绕）
    function [18:0] star_xy;         // {x[18:9], y[8:0]}
        input [5:0] i;
        begin
            case (i)
                6'd0: star_xy = 19'd314470;   // (614,102)
                6'd1: star_xy = 19'd45225;    // (88,169)
                6'd2: star_xy = 19'd15982;    // (31,110)
                6'd3: star_xy = 19'd282055;   // (550,455)
                6'd4: star_xy = 19'd27949;    // (54,269)
                6'd5: star_xy = 19'd208733;   // (407,405)
                6'd6: star_xy = 19'd106661;   // (208,189)
                6'd7: star_xy = 19'd29574;    // (57,302)
                6'd8: star_xy = 19'd27290;    // (53,162)
                6'd9: star_xy = 19'd152649;   // (298,73)
                6'd10: star_xy = 19'd140033;  // (273,265)
                6'd11: star_xy = 19'd234558;  // (458,86)
                6'd12: star_xy = 19'd91613;   // (178,365)
                6'd13: star_xy = 19'd248069;  // (484,357)
                6'd14: star_xy = 19'd272913;  // (533,129)
                6'd15: star_xy = 19'd169536;  // (331,160)
                6'd16: star_xy = 19'd122945;  // (240,225)
                6'd17: star_xy = 19'd192786;  // (376,290)
                6'd18: star_xy = 19'd309223;  // (603,455)
                6'd19: star_xy = 19'd131581;  // (257,37)
                6'd20: star_xy = 19'd103753;  // (202,329)
                6'd21: star_xy = 19'd35869;   // (70,29)
                6'd22: star_xy = 19'd196635;  // (384,83)
                6'd23: star_xy = 19'd294864;  // (576,96)
                6'd24: star_xy = 19'd236754;  // (462,210)
                6'd25: star_xy = 19'd117679;  // (230,79)
                6'd26: star_xy = 19'd137311;  // (268,223)
                6'd27: star_xy = 19'd321972;  // (629,92)
                6'd28: star_xy = 19'd64176;   // (125,136)
                6'd29: star_xy = 19'd182032;  // (355,312)
                6'd30: star_xy = 19'd47783;   // (93,175)
                6'd31: star_xy = 19'd265180;  // (518,244)
                6'd32: star_xy = 19'd74642;   // (145,402)
                6'd33: star_xy = 19'd289884;  // (566,308)
                6'd34: star_xy = 19'd214879;  // (420,59)
                6'd35: star_xy = 19'd300835;  // (588,227)
                6'd36: star_xy = 19'd253501;  // (495,221)
                6'd37: star_xy = 19'd10615;   // (20,215)
                6'd38: star_xy = 19'd252511;  // (493,399)
                6'd39: star_xy = 19'd125222;  // (245,22)
                default: star_xy = 19'd0;
            endcase
        end
    endfunction

    // 相位/步长寄存器：每帧 vs 沿并行 +step（1/2/3 → 周期 0.7~2.1s）
    reg [5:0] star_ph [0:39];
    reg [1:0] star_st [0:39];
    integer si;
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            for (si = 0; si < 40; si = si + 1) begin
                star_ph[si] <= si * 13;                        // 截断即 mod 64
                star_st[si] <= ((si % 4) == 3) ? 2'd1 : ((si % 4) + 2'd1);
            end
        end
        else if (vs_rise && star_on) begin
            for (si = 0; si < 40; si = si + 1)
                star_ph[si] <= star_ph[si] + {4'd0, star_st[si]};
        end
    end

    // 星空填充（fsub==1 算，fsub 2..6 写 dx=-2..+2）
    wire        star_fill = star_on && (fi < 6'd40);
    wire [18:0] sxy       = star_xy(fi);
    wire [9:0]  st_x      = sxy[18:9];
    wire [8:0]  st_y      = sxy[8:0];
    wire signed [10:0] st_dy = $signed({2'b00, st_y}) - $signed({1'b0, row_fill});
    wire [1:0]  st_dya = (st_dy == 11'sd0)  ? 2'd0 :
                         ((st_dy == 11'sd1) || (st_dy == -11'sd1)) ? 2'd1 : 2'd2;
    wire        st_dy0 = (st_dy == 11'sd0);
    wire        st_hit = (st_dy >= -11'sd2) && (st_dy <= 11'sd2) &&
                         (star_ph[fi][5:4] != 2'd2);           // 灭段不出

    reg        s_hit_r, s_dy0_r;
    reg [9:0]  s_x_r;
    reg [1:0]  s_dya_r, s_q_r;
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            s_hit_r <= 1'b0;
            s_dy0_r <= 1'b0;
            s_x_r   <= 10'd0;
            s_dya_r <= 2'd0;
            s_q_r   <= 2'd0;
        end
        else if (fill_active && star_fill && (fsub == 3'd1)) begin
            s_hit_r <= st_hit;
            s_dy0_r <= st_dy0;
            s_x_r   <= st_x;
            s_dya_r <= st_dya;
            s_q_r   <= star_ph[fi][5:4];
        end
    end

    // 写拍：dx = -2..+2（fsub 2..6），十字臂条件 dx==0(竖臂) 或 dy==0(横臂)
    wire [2:0] st_adx = (fsub <= 3'd4) ? (3'd4 - fsub) : (fsub - 3'd4);
    wire [1:0] st_role = (st_adx[1:0] > s_dya_r) ? st_adx[1:0] : s_dya_r;
    wire [3:0] st_val = (s_q_r == 2'd0) ?
                        ((st_role == 2'd0) ? 4'd1 :      // 亮段：中心 FF
                         (st_role == 2'd1) ? 4'd5 : 4'd4)//       内臂 D0 外臂 B8
                                       :
                        ((st_role == 2'd0) ? 4'd5 :      // 微段：中心 D0
                         (st_role == 2'd1) ? 4'd4 : 4'd0);//       内臂 B8 外臂灭
    wire [3:0] st_dxv = (fsub == 3'd2) ? 4'd14 : (fsub == 3'd3) ? 4'd15 :
                        (fsub == 3'd4) ? 4'd0  : (fsub == 3'd5) ? 4'd1 : 4'd2;
    wire [9:0] st_wa  = s_x_r + {{6{st_dxv[3]}}, st_dxv};  // 补码符号扩展：-2/-1/0/+1/+2
    wire st_dx0 = (fsub == 3'd4);                         // dx==0（竖臂/中心）
    wire star_fill_hold = star_on && s_hit_r && (st_dx0 || s_dy0_r) &&
                          (fsub >= 3'd2) && (fsub <= 3'd6);
    wire st_we = star_fill_hold && (st_val != 4'd0);

    // 子相位 2/3/4：雪花写命中像素（-1 / 0 / +1，按档位窗口）
    wire wr_m1 = hit_r && (fsub == 3'd2) && (cls_r != 2'd0);
    wire wr_c0 = hit_r && (fsub == 3'd3);
    wire wr_p1 = hit_r && (fsub == 3'd4) && (cls_r == 2'd2);

    wire [9:0] wa_m1 = (fx_r == 10'd0)   ? 10'd639 : (fx_r - 10'd1);
    wire [9:0] wa_p1 = (fx_r == 10'd639) ? 10'd0  : (fx_r + 10'd1);

    assign lwe = st_we || wr_m1 || wr_c0 || wr_p1;
    assign lwa = st_we ? {row_fill[0], st_wa} :
                 {row_fill[0], wr_m1 ? wa_m1 : (wr_p1 ? wa_p1 : fx_r)};
    assign lwd = st_we ? st_val : ({1'b0, cls_r} + 4'd1);

    // 读地址：初始化/更新遍与填充遍分时复用读口
    wire init_rd = in_init && snow_on && (ph < 10'd64) && (ph[0] == 1'b0);
    assign fra_r = (upd_rd || init_rd) ? ui : fi;

    // 写：初始化遍（0..63 每偶拍写一片）；更新遍奇拍写回
    assign fwe = (in_init && snow_on && (ph < 10'd64)) || upd_wr;
    assign fwa = in_init ? ph[5:0] : ui;
    assign fwd = in_init ? flake_init(ph[5:0]) : st_next;

    // init_done / ready
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            init_done <= 1'b0;
            ready     <= 1'b0;
        end
        else begin
            if (in_init && snow_on && (ph == 10'd63))
                init_done <= 1'b1;
            if (any_on && de_rise)
                ready <= 1'b1;          // 首帧消隐期已填第 0 行，首行 de 后参与融合
        end
    end

    // ---------------- 像素融合（1 拍流水对齐 BRAM 读延迟） ----------------
    reg        d_de, d_vs;
    reg [23:0] d_rgb;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            d_de  <= 1'b0;
            d_vs  <= 1'b0;
            d_rgb <= 24'd0;
        end
        else begin
            d_de  <= I_de;
            d_vs  <= I_vs;
            d_rgb <= I_rgb;
        end
    end

    // 灰度索引表：1=FF 2=E8 3=D8（雪花三档）；4=B8 5=D0（星空外臂/内臂）
    wire [7:0] g_val = (lrd == 4'd1) ? 8'hFF :
                       (lrd == 4'd2) ? 8'hE8 :
                       (lrd == 4'd3) ? 8'hD8 :
                       (lrd == 4'd4) ? 8'hB8 :
                       (lrd == 4'd5) ? 8'hD0 : 8'h00;

    function [7:0] max8;
        input [7:0] a, b;
        begin
            max8 = (a > b) ? a : b;
        end
    endfunction

    wire flake_here = (lrd != 4'd0) && ready && any_on;

    assign O_vs  = d_vs;
    assign O_de  = d_de;
    assign O_rgb = flake_here ? {max8(d_rgb[23:16], g_val),
                                 max8(d_rgb[15:8],  g_val),
                                 max8(d_rgb[7:0],   g_val)}
                              : d_rgb;

endmodule
