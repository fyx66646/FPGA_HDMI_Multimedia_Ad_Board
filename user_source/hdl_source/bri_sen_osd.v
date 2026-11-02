// ============================================================================
// bri_sen_osd : 五项参数菜单调节 + 中文 OSD（video_clk 域）
// 控制（2026-10-24 重构：K3 菜单计数器选择；2026-10-28 起 SW4=音频可视化开关）：
//   SW4 高=显示频谱可视化、低=隐藏；拨动后弹 2 秒提示"音频可视化已显示/已隐藏"
//   K3  每按一次菜单计数器 ±1（SW3 定方向：低=向下 +1，高=向上 -1；0..6 回绕）：
//     0 对比度   1 亮度   2 饱和度   3 音量   4 音调   5 转场特效   6 粒子特效
//   SW2 高 = 档位 +1，低 = 档位 -1
//   K4  每按一次，按 SW2 方向调整 K3 选中的项（转场特效 0..4 循环回绕）
//       转场特效的修改以 toggle 脉冲（O_eff_up/O_eff_dn）送到 sd_card_bmp
//       的 effect_cnt，本模块同时维护本地镜像 eff_lvl 用于显示，两侧同源计数。
// 档位表（均默认 100%/1.0x/特效0）：
//   对比度 11 档  50%~150%（10% 步进）    out = clamp(((rgb-128)*c)>>8+128)
//   亮度   15 档  10%~150%（10% 步进）    画面乘 gain = pct*256/100
//   饱和度 21 档  -100%~+100%（10% 步进，0=原样，-100=纯黑白）
//     Y=(77R+150G+29B)>>8，C'=Y+((C-Y)*gain)>>8，gain=档位表 0..512
//   音量   11 档  0%~100%  （10% 步进）   O_vol_gain = pct*256/100 → 播放器输出乘
//   音调   16 档  0.5x~2.0x（0.1 步进）   O_rate(8.8 定点) → 播放器样本消耗速率
//   转场特效 6 档 0无/1淡入淡出/2滑动/3栅格化/4擦除/5粒子
//   粒子特效 3 档 0无/2雪花(64 片)/3星空(40 星)，默认无（K4 循环切换）
// OSD（操作后显示 2 秒自动隐藏）：
//   菜单 OSD（K3/K4 或 SW2 拨动触发）：左上角七行中文框（x 8..151、y 8..151，9 单元）：
//     对比度 100% / 亮度 100% / 饱和度 0 / 音量 100% / 音调 1.0x / 转场特效:无 / 粒子特效:无
//     当前 K3 选中的行高亮（白字亮底），其余行灰字暗底；饱和度在亮度与音量之间。
//   SW1 OSD（拨动 SW1 触发）：左上角单行框（x 8..135、y 8..31）：
//     "自动连播：开/关"，白字亮底。
//   SW4 OSD（拨动 SW4 触发）：顶部中段单行框（x 168..295、y 8..31）：
//     "音频可视化已显示/已隐藏"，白字亮底，与菜单/分辨率空间不重叠。
//   分辨率 OSD（图片提交触发，右上角 x 504..632、两行 y 8..51）：
//     上行"分辨率"居中、下行 640x480/1024x768/1280x960，2 秒自动隐藏。
//   汉字用 osd_cn_rom（对0 比1 度2 亮3 音4 量5 调6 自7 动8 连9 播A 开B 关C
//     转D 场E 特F 效10 无11 淡12 入13 出14 滑15 擦16 除17 栅18 格19 化1A
//     粒1B 子1C 雪D1 花1E 小1F 盛20 星21 空22 分23 辨24 率25 饱26 和27
//     频28 可29 视2A 已2B 显2C 示2D 隐2E 藏2F），
//   数字/.x/%/冒号 用 font8x16_rom（8x16 居中在单元第 4..11 列）。
// ============================================================================
module bri_sen_osd (
    input  wire        I_clk,          // video_clk 25MHz
    input  wire        I_rst,          // 高有效

    input  wire        I_vs,
    input  wire        I_de,
    input  wire [23:0] I_rgb,

    input  wire        I_sw1,          // 自动轮播开关（高=自动，拨动触发 SW1 OSD）
    input  wire        I_sw2,          // 高=往上，低=往下（异步，内部同步）
    input  wire        I_sw3,          // 菜单滚动方向：低=K3 向下(+1)，高=K3 向上(-1)
    input  wire        I_sw4,          // 音频可视化开关（高=显示，低=隐藏，拨动触发 SW4 OSD）
    input  wire        I_k4,           // 每按一次调整选中项一档（异步，内部消抖）
    input  wire        I_k3,           // 每按一次菜单计数器 +1：0对比度..5粒子特效

    input  wire        I_img_tgl,      // 图片提交 toggle（sd_card_clk 域，触发分辨率 OSD）
    input  wire [1:0]  I_img_tier,     // 源图分辨率挡 0/1/2（准静态，提交时更新）

    output wire [9:0]  O_vol_gain,     // 音量增益 0..256（供音频播放器，准静态）
    output wire [9:0]  O_rate,         // 倍速 8.8 定点 128..512 = 0.5x..2.0x
    output reg         O_eff_up,       // 转场特效 +1 事件（toggle，sd_card_bmp 边沿检测）
    output reg         O_eff_dn,       // 转场特效 -1 事件（toggle）
    output wire [1:0]  O_snow_lvl,     // 粒子特效档 0无/2雪花/3星空（同域 snow_overlay 用）
    output wire        O_viz_show,     // 音频可视化开关电平（同步后，viz_overlay 显示门控）

    output wire        O_vs,
    output wire        O_de,
    output reg  [23:0] O_rgb
);

    localparam [3:0] CON_MAX = 4'd10;      // 对比度 0..10 → 50..150%
    localparam [3:0] BRI_MAX = 4'd14;      // 亮度   0..14 → 10..150%
    localparam [3:0] VOL_MAX = 4'd10;      // 音量   0..10 → 0..100%
    localparam [3:0] RAT_MAX = 4'd15;      // 音调   0..15 → 0.5..2.0x

    // ---------------- 开关同步 + K4/K3 消抖 ----------------
    reg sw1_s0, sw1_s1, sw2_s0, sw2_s1, sw3_s0, sw3_s1, sw4_s0, sw4_s1;
    reg itgl_s0, itgl_s1, itgl_s2;
    reg [1:0] itier_s0, itier_s1;
    wire k4_press;
    wire k3_press;
    wire img_commit_edge;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            sw1_s0 <= 1'b0; sw1_s1 <= 1'b0;
            sw2_s0 <= 1'b0; sw2_s1 <= 1'b0;
            sw3_s0 <= 1'b0; sw3_s1 <= 1'b0;
            sw4_s0 <= 1'b0; sw4_s1 <= 1'b0;
            itgl_s0 <= 1'b0; itgl_s1 <= 1'b0; itgl_s2 <= 1'b0;
            itier_s0 <= 2'd0; itier_s1 <= 2'd0;
        end
        else begin
            sw1_s0 <= I_sw1; sw1_s1 <= sw1_s0;
            sw2_s0 <= I_sw2; sw2_s1 <= sw2_s0;
            sw3_s0 <= I_sw3; sw3_s1 <= sw3_s0;
            sw4_s0 <= I_sw4; sw4_s1 <= sw4_s0;
            itgl_s0 <= I_img_tgl; itgl_s1 <= itgl_s0; itgl_s2 <= itgl_s1;
            itier_s0 <= I_img_tier; itier_s1 <= itier_s0;
        end
    end

    // 3 级同步 + [2]^[1] 边沿：脉冲拍上 itier_s1 必为新值
    // （2 级 + [0]^[1] 边沿会提前一拍触发、锁到上一张图的 tier——曾致错位）
    assign img_commit_edge = itgl_s2 ^ itgl_s1;

    key_press_debounce #(
        .CLK_FREQ_HZ (25_000_000),
        .DEBOUNCE_MS (20)
    ) u_k4 (
        .clk        (I_clk),
        .rst        (I_rst),
        .button_in  (I_k4),
        .press_pulse(k4_press)
    );

    key_press_debounce #(
        .CLK_FREQ_HZ (25_000_000),
        .DEBOUNCE_MS (20)
    ) u_k3 (
        .clk        (I_clk),
        .rst        (I_rst),
        .button_in  (I_k3),
        .press_pulse(k3_press)
    );

    // ---------------- 菜单选择计数器（K3：每按一次 ±1，方向由 SW3 定，0..6 回绕） ----------------
    // SW3 低 = 向下(+1)，SW3 高 = 向上(-1)
    // 0 对比度 1 亮度 2 饱和度 3 音量 4 音调 5 转场特效 6 粒子特效
    reg [2:0] menu_sel;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst)
            menu_sel <= 3'd0;
        else if (k3_press)
            menu_sel <= sw3_s1 ? ((menu_sel == 3'd0) ? 3'd6 : (menu_sel - 3'd1))
                               : ((menu_sel == 3'd6) ? 3'd0 : (menu_sel + 3'd1));
    end

    // 拨动 SW2/SW1（任一）也触发一次 OSD 显示；SW4 拨动触发音频可视化提示
    reg       sw1_prev, sw2_prev, sw4_prev;
    wire      sw1_change = (sw1_s1 != sw1_prev);
    wire      sw2_change = (sw2_s1 != sw2_prev);
    wire      sw4_change = (sw4_s1 != sw4_prev);

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            sw1_prev <= 1'b0;
            sw2_prev <= 1'b0;
            sw4_prev <= 1'b0;
        end
        else begin
            sw1_prev <= sw1_s1;
            sw2_prev <= sw2_s1;
            sw4_prev <= sw4_s1;
        end
    end

    // ---------------- 档位寄存器（默认 100% / 1.0x / 饱和 0 / 特效0 / 粒子无） ----------------
    reg [3:0] con_lv, bri_lv, vol_lv, rat_lv;
    reg [4:0] sat_lv;                      // 饱和度 0..20 → -100..+100%（默认 10=0）
    reg [2:0] eff_lvl;                     // 转场特效镜像（与 sd_card_bmp 的 effect_cnt 同源计数）
    reg [1:0] snow_lv;                     // 粒子特效档 0无/2雪花/3星空

    localparam [4:0] SAT_MAX = 5'd20;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            con_lv   <= 4'd5;
            bri_lv   <= 4'd9;
            sat_lv   <= 5'd10;
            vol_lv   <= 4'd10;
            rat_lv   <= 4'd5;
            eff_lvl  <= 3'd0;
            snow_lv  <= 2'd0;
            O_eff_up <= 1'b0;
            O_eff_dn <= 1'b0;
        end
        else if (k4_press) begin
            if (menu_sel == 3'd6) begin
                // 粒子特效：无 → 雪花(64片) → 星空(40星) → 无 循环
                snow_lv <= (snow_lv == 2'd0) ? 2'd2 :
                           (snow_lv == 2'd2) ? 2'd3 : 2'd0;
            end
            else if (menu_sel == 3'd5) begin
                // 转场特效：SW2 方向 +1/-1，0..5 循环回绕（5→0、0→5）；
                // 每次按下值必变并发 toggle，与 sd 侧同源保持镜像一致
                if (sw2_s1) begin
                    eff_lvl  <= (eff_lvl == 3'd5) ? 3'd0 : (eff_lvl + 3'd1);
                    O_eff_up <= ~O_eff_up;
                end
                else begin
                    eff_lvl  <= (eff_lvl == 3'd0) ? 3'd5 : (eff_lvl - 3'd1);
                    O_eff_dn <= ~O_eff_dn;
                end
            end
            else begin
                case (menu_sel[2:0])
                    3'd0: if (sw2_s1) begin if (con_lv < CON_MAX) con_lv <= con_lv + 4'd1; end
                          else        begin if (con_lv > 4'd0)    con_lv <= con_lv - 4'd1; end
                    3'd1: if (sw2_s1) begin if (bri_lv < BRI_MAX) bri_lv <= bri_lv + 4'd1; end
                          else        begin if (bri_lv > 4'd0)    bri_lv <= bri_lv - 4'd1; end
                    3'd2: if (sw2_s1) begin if (sat_lv < SAT_MAX) sat_lv <= sat_lv + 5'd1; end
                          else        begin if (sat_lv > 5'd0)    sat_lv <= sat_lv - 5'd1; end
                    3'd3: if (sw2_s1) begin if (vol_lv < VOL_MAX) vol_lv <= vol_lv + 4'd1; end
                          else        begin if (vol_lv > 4'd0)    vol_lv <= vol_lv - 4'd1; end
                    default: // 3'd4 音调
                          if (sw2_s1) begin if (rat_lv < RAT_MAX) rat_lv <= rat_lv + 4'd1; end
                          else        begin if (rat_lv > 4'd0)    rat_lv <= rat_lv - 4'd1; end
                endcase
            end
        end
    end

    // ---------------- OSD 显示控制：双独立通道，各显示 2 秒自动隐藏 ----------------
    // 菜单通道（左侧）：六行菜单/SW1 连播提示，按键/SW1/SW2 触发
    // 分辨率通道（右上角）：图片提交触发——与菜单空间不重叠，可同时显示
    // （2026-10-26 修复：原单通道 osd_mode 被分辨率显示顶掉菜单）
    reg       show_menu;
    reg [25:0] menu_timer;       // 2s @ 25MHz = 50,000,000
    reg       menu_mode;         // 0=六行菜单  1=SW1 自动连播（触发时锁定）
    reg       show_res;
    reg [25:0] res_timer;
    reg [1:0] res_lv;            // 分辨率挡（提交沿锁存，tier 在加载早期已稳定）
    reg       show_viz;          // SW4 提示通道（顶部中段，与菜单/分辨率空间不重叠）
    reg [25:0] viz_timer;        // 2s @ 25MHz
    reg       viz_on;            // 拨动后的新状态：1=已显示 0=已隐藏

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            show_menu  <= 1'b0;
            menu_timer <= 26'd0;
            menu_mode  <= 1'b0;
            show_res   <= 1'b0;
            res_timer  <= 26'd0;
            res_lv     <= 2'd0;
            show_viz   <= 1'b0;
            viz_timer  <= 26'd0;
            viz_on     <= 1'b0;
        end
        else begin
            if (img_commit_edge) begin
                res_lv    <= itier_s1;
                show_res  <= 1'b1;
                res_timer <= 26'd50_000_000;
            end
            else if (show_res) begin
                if (res_timer != 26'd0)
                    res_timer <= res_timer - 26'd1;
                else
                    show_res <= 1'b0;
            end

            // SW4 通道：拨动触发，锁定拨动后的新电平作为提示内容
            if (sw4_change) begin
                show_viz  <= 1'b1;
                viz_timer <= 26'd50_000_000;
                viz_on    <= sw4_s1;
            end
            else if (show_viz) begin
                if (viz_timer != 26'd0)
                    viz_timer <= viz_timer - 26'd1;
                else
                    show_viz <= 1'b0;
            end

            if (k4_press || k3_press || sw2_change || sw1_change) begin
                show_menu  <= 1'b1;
                menu_timer <= 26'd50_000_000;
                menu_mode  <= sw1_change ? 1'b1 : 1'b0;
            end
            else if (show_menu) begin
                if (menu_timer != 26'd0)
                    menu_timer <= menu_timer - 26'd1;
                else
                    show_menu <= 1'b0;
            end
        end
    end

    // ---------------- 增益表（gain = pct*256/100） ----------------
    function [8:0] bri_gain;        // 亮度 10..150%
        input [3:0] lv;
        begin
            case (lv)
                4'd0 : bri_gain = 9'd26;   4'd1 : bri_gain = 9'd51;
                4'd2 : bri_gain = 9'd77;   4'd3 : bri_gain = 9'd102;
                4'd4 : bri_gain = 9'd128;  4'd5 : bri_gain = 9'd154;
                4'd6 : bri_gain = 9'd179;  4'd7 : bri_gain = 9'd205;
                4'd8 : bri_gain = 9'd230;  4'd9 : bri_gain = 9'd256;
                4'd10: bri_gain = 9'd282;  4'd11: bri_gain = 9'd307;
                4'd12: bri_gain = 9'd333;  4'd13: bri_gain = 9'd358;
                default: bri_gain = 9'd384;
            endcase
        end
    endfunction

    function [8:0] con_gain;        // 对比度 50..150%
        input [3:0] lv;
        begin
            case (lv)
                4'd0 : con_gain = 9'd128;  4'd1 : con_gain = 9'd154;
                4'd2 : con_gain = 9'd179;  4'd3 : con_gain = 9'd205;
                4'd4 : con_gain = 9'd230;  4'd5 : con_gain = 9'd256;
                4'd6 : con_gain = 9'd282;  4'd7 : con_gain = 9'd307;
                4'd8 : con_gain = 9'd333;  4'd9 : con_gain = 9'd358;
                default: con_gain = 9'd384;
            endcase
        end
    endfunction

    function [9:0] vol_gain;        // 音量 0..100%
        input [3:0] lv;
        begin
            case (lv)
                4'd0 : vol_gain = 10'd0;   4'd1 : vol_gain = 10'd26;
                4'd2 : vol_gain = 10'd51;  4'd3 : vol_gain = 10'd77;
                4'd4 : vol_gain = 10'd102; 4'd5 : vol_gain = 10'd128;
                4'd6 : vol_gain = 10'd154; 4'd7 : vol_gain = 10'd179;
                4'd8 : vol_gain = 10'd205; 4'd9 : vol_gain = 10'd230;
                default: vol_gain = 10'd256;
            endcase
        end
    endfunction

    function [9:0] rate_gain;       // 音调 0.5..2.0x（8.8 定点）
        input [3:0] lv;
        begin
            case (lv)
                4'd0 : rate_gain = 10'd128;  // 0.5x
                4'd1 : rate_gain = 10'd154;  // 0.6x
                4'd2 : rate_gain = 10'd179;  // 0.7x
                4'd3 : rate_gain = 10'd205;  // 0.8x
                4'd4 : rate_gain = 10'd230;  // 0.9x
                4'd5 : rate_gain = 10'd256;  // 1.0x
                4'd6 : rate_gain = 10'd282;  // 1.1x
                4'd7 : rate_gain = 10'd307;  // 1.2x
                4'd8 : rate_gain = 10'd333;  // 1.3x
                4'd9 : rate_gain = 10'd358;  // 1.4x
                4'd10: rate_gain = 10'd384;  // 1.5x
                4'd11: rate_gain = 10'd410;  // 1.6x
                4'd12: rate_gain = 10'd435;  // 1.7x
                4'd13: rate_gain = 10'd461;  // 1.8x
                4'd14: rate_gain = 10'd486;  // 1.9x
                default: rate_gain = 10'd512;// 2.0x
            endcase
        end
    endfunction

    function [9:0] sat_gain;       // 饱和度 -100..100%（档 0..20）→ Q8 增益 0..512
        input [4:0] lv;
        begin
            case (lv)
                5'd0 : sat_gain = 10'd0;    5'd1 : sat_gain = 10'd26;
                5'd2 : sat_gain = 10'd51;   5'd3 : sat_gain = 10'd77;
                5'd4 : sat_gain = 10'd102;  5'd5 : sat_gain = 10'd128;
                5'd6 : sat_gain = 10'd154;  5'd7 : sat_gain = 10'd179;
                5'd8 : sat_gain = 10'd205;  5'd9 : sat_gain = 10'd230;
                5'd10: sat_gain = 10'd256;  5'd11: sat_gain = 10'd282;
                5'd12: sat_gain = 10'd307;  5'd13: sat_gain = 10'd333;
                5'd14: sat_gain = 10'd358;  5'd15: sat_gain = 10'd384;
                5'd16: sat_gain = 10'd410;  5'd17: sat_gain = 10'd435;
                5'd18: sat_gain = 10'd461;  5'd19: sat_gain = 10'd486;
                default: sat_gain = 10'd512;
            endcase
        end
    endfunction

    wire [8:0] gain  = bri_gain(bri_lv);
    wire [8:0] cgain = con_gain(con_lv);
    wire [9:0] sgain = sat_gain(sat_lv);
    assign O_vol_gain = vol_gain(vol_lv);
    assign O_rate     = rate_gain(rat_lv);
    assign O_snow_lvl = snow_lv;
    assign O_viz_show = sw4_s1;

    // ---------------- 帧 / 行 / 像素计数 ----------------
    reg        vs_d, de_d;
    reg [10:0] x_cnt, y_cnt;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            vs_d  <= 1'b0;
            de_d  <= 1'b0;
            x_cnt <= 11'd0;
            y_cnt <= 11'd0;
        end
        else begin
            vs_d <= I_vs;
            de_d <= I_de;

            if (vs_d != I_vs)
                y_cnt <= 11'd0;
            else if (I_de && (x_cnt == 11'd639))
                y_cnt <= y_cnt + 11'd1;

            if (I_de)
                x_cnt <= (x_cnt == 11'd639) ? 11'd0 : (x_cnt + 11'd1);
            else
                x_cnt <= 11'd0;
        end
    end

    // ---------------- OSD 数值字符（查表避免除法） ----------------
    // 对比度/亮度 百位：>=100% 为 '1' 否则空；音调整数位：0.*/1.*/2.0
    function [7:0] d100;
        input [3:0] lv;
        input [3:0] hund_at;        // 达到 100% 的档位
        begin
            d100 = (lv >= hund_at) ? 8'h31 : 8'h20;
        end
    endfunction

    function [7:0] con_d10;         // 对比度 50..150% 十位
        input [3:0] lv;
        begin
            case (lv)
                4'd0 : con_d10 = 8'h35;  4'd1 : con_d10 = 8'h36;
                4'd2 : con_d10 = 8'h37;  4'd3 : con_d10 = 8'h38;
                4'd4 : con_d10 = 8'h39;  4'd5 : con_d10 = 8'h30;
                4'd6 : con_d10 = 8'h31;  4'd7 : con_d10 = 8'h32;
                4'd8 : con_d10 = 8'h33;  4'd9 : con_d10 = 8'h34;
                default: con_d10 = 8'h35;
            endcase
        end
    endfunction

    function [7:0] bri_d10;         // 亮度 10..150% 十位
        input [3:0] lv;
        begin
            case (lv)
                4'd0 : bri_d10 = 8'h31;  4'd1 : bri_d10 = 8'h32;
                4'd2 : bri_d10 = 8'h33;  4'd3 : bri_d10 = 8'h34;
                4'd4 : bri_d10 = 8'h35;  4'd5 : bri_d10 = 8'h36;
                4'd6 : bri_d10 = 8'h37;  4'd7 : bri_d10 = 8'h38;
                4'd8 : bri_d10 = 8'h39;  4'd9 : bri_d10 = 8'h30;
                4'd10: bri_d10 = 8'h31;  4'd11: bri_d10 = 8'h32;
                4'd12: bri_d10 = 8'h33;  4'd13: bri_d10 = 8'h34;
                default: bri_d10 = 8'h35;
            endcase
        end
    endfunction

    function [7:0] vol_d10;         // 音量 0..100% 十位
        input [3:0] lv;
        begin
            case (lv)
                4'd0 : vol_d10 = 8'h30;  4'd1 : vol_d10 = 8'h31;
                4'd2 : vol_d10 = 8'h32;  4'd3 : vol_d10 = 8'h33;
                4'd4 : vol_d10 = 8'h34;  4'd5 : vol_d10 = 8'h35;
                4'd6 : vol_d10 = 8'h36;  4'd7 : vol_d10 = 8'h37;
                4'd8 : vol_d10 = 8'h38;  4'd9 : vol_d10 = 8'h39;
                default: vol_d10 = 8'h30;
            endcase
        end
    endfunction

    function [7:0] rat_int;         // 音调整数位 0/1/2
        input [3:0] lv;
        begin
            rat_int = (lv >= 4'd15) ? 8'h32 : ((lv >= 4'd5) ? 8'h31 : 8'h30);
        end
    endfunction

    function [7:0] rat_frac;        // 音调十分位 (5+lv)%10
        input [3:0] lv;
        begin
            case (lv)
                4'd0 : rat_frac = 8'h35;  4'd1 : rat_frac = 8'h36;
                4'd2 : rat_frac = 8'h37;  4'd3 : rat_frac = 8'h38;
                4'd4 : rat_frac = 8'h39;  4'd5 : rat_frac = 8'h30;
                4'd6 : rat_frac = 8'h31;  4'd7 : rat_frac = 8'h32;
                4'd8 : rat_frac = 8'h33;  4'd9 : rat_frac = 8'h34;
                4'd10: rat_frac = 8'h35;  4'd11: rat_frac = 8'h36;
                4'd12: rat_frac = 8'h37;  4'd13: rat_frac = 8'h38;
                4'd14: rat_frac = 8'h39;  default: rat_frac = 8'h30;
            endcase
        end
    endfunction

    // ---------------- OSD 单元码表（五行参数，各 8 单元 x 16px） ----------------
    // 汉字码：对0x80 比0x81 度0x82 亮0x83 音0x84 量0x85 调0x86 饱0xA6 和0xA7（osd_cn_rom）
    // 行 0 "对比度 100%"   行 1 "亮度   100%"   行 2 "饱和度  -60"
    // 行 3 "音量   100%"   行 4 "音调   1.0x"（数值位五行对齐）
    function [7:0] osd_code;
        input [3:0] ci;
        input [2:0] ln;
        input [3:0] c_lv, b_lv, v_lv, r_lv;
        input [4:0] s_lv;
        reg [4:0] s_idx;                       // |s_lv-10| = |饱和值|/10，0..10
        begin
            case (ln)
                3'd0: begin                              // 对比度
                    case (ci)
                        3'd0: osd_code = 8'h80;
                        3'd1: osd_code = 8'h81;
                        3'd2: osd_code = 8'h82;
                        3'd3: osd_code = 8'h20;
                        4'd4: osd_code = d100(c_lv, 4'd5);
                        4'd5: osd_code = con_d10(c_lv);
                        4'd6: osd_code = 8'h30;
                        4'd7: osd_code = 8'h25;       // '%'
                        default: osd_code = 8'h20;   // 第 9 单元空白
                    endcase
                end
                3'd1: begin                              // 亮度
                    case (ci)
                        3'd0: osd_code = 8'h83;
                        3'd1: osd_code = 8'h82;
                        3'd2: osd_code = 8'h20;
                        3'd3: osd_code = 8'h20;
                        4'd4: osd_code = d100(b_lv, 4'd9);
                        4'd5: osd_code = bri_d10(b_lv);
                        4'd6: osd_code = 8'h30;
                        4'd7: osd_code = 8'h25;
                        default: osd_code = 8'h20;
                    endcase
                end
                3'd2: begin                              // 饱和度 -100..100
                    s_idx = (s_lv >= 5'd10) ? (s_lv - 5'd10) : (5'd10 - s_lv);
                    case (ci)
                        3'd0: osd_code = 8'hA6;          // 饱
                        3'd1: osd_code = 8'hA7;          // 和
                        3'd2: osd_code = 8'h82;          // 度
                        3'd3: osd_code = 8'h20;
                        4'd4: osd_code = (s_lv < 5'd10) ? 8'h2D : 8'h20;  // '-'
                        4'd5: osd_code = (s_idx == 5'd10) ? 8'h31 : 8'h20;
                        4'd6: osd_code = (s_idx == 5'd0)  ? 8'h20 :
                                         (s_idx == 5'd10) ? 8'h30 :   // 100 的十位
                                         (8'h30 + s_idx[3:0]);        // 十位
                        4'd7: osd_code = 8'h30;          // 个位恒 0（10% 步进）
                        default: osd_code = 8'h20;
                    endcase
                end
                3'd3: begin                              // 音量
                    case (ci)
                        3'd0: osd_code = 8'h84;
                        3'd1: osd_code = 8'h85;
                        3'd2: osd_code = 8'h20;
                        3'd3: osd_code = 8'h20;
                        4'd4: osd_code = d100(v_lv, 4'd10);
                        4'd5: osd_code = vol_d10(v_lv);
                        4'd6: osd_code = 8'h30;
                        4'd7: osd_code = 8'h25;
                        default: osd_code = 8'h20;
                    endcase
                end
                default: begin                           // 音调
                    case (ci)
                        3'd0: osd_code = 8'h84;
                        3'd1: osd_code = 8'h86;
                        3'd2: osd_code = 8'h20;
                        3'd3: osd_code = 8'h20;
                        4'd4: osd_code = rat_int(r_lv);
                        4'd5: osd_code = 8'h2E;          // '.'
                        4'd6: osd_code = rat_frac(r_lv);
                        4'd7: osd_code = 8'h78;          // 'x'
                        default: osd_code = 8'h20;
                    endcase
                end
            endcase
        end
    endfunction

    // ---------------- SW1 OSD 码表（单行 8 单元："自动连播：开/关"） ----------------
    // 自0x87 动0x88 连0x89 播0x8A 开0x8B 关0x8C
    function [7:0] sw1_code;
        input [3:0] ci;             // 4 位：与调用方一致，防未来加宽时再踩截断坑
        input       s;
        begin
            case (ci)
                3'd0: sw1_code = 8'h87;                  // 自
                3'd1: sw1_code = 8'h88;                  // 动
                3'd2: sw1_code = 8'h89;                  // 连
                3'd3: sw1_code = 8'h8A;                  // 播
                3'd4: sw1_code = 8'h3A;                  // ':'
                3'd5: sw1_code = 8'h20;                  // 空
                3'd6: sw1_code = s ? 8'h8B : 8'h8C;      // 开 / 关
                default: sw1_code = 8'h20;
            endcase
        end
    endfunction

    // ---------------- 特效 OSD 码表（单行 9 单元："转场特效:XXX"） ----------------
    // 转0x8D 场0x8E 特0x8F 效0x90 无0x91 淡0x92 入0x93 出0x94 滑0x95（动0x88）
    // 擦0x96 除0x97 栅0x98 格0x99 化0x9A 粒0x9B 子0x9C
    function [7:0] eff_code;
        input [3:0] ci;
        input [2:0] e;
        begin
            case (ci)
                3'd0: eff_code = 8'h8D;                  // 转
                3'd1: eff_code = 8'h8E;                  // 场
                3'd2: eff_code = 8'h8F;                  // 特
                3'd3: eff_code = 8'h90;                  // 效
                3'd4: eff_code = 8'h3A;                  // ':'
                3'd5: begin
                    case (e)
                        3'd0: eff_code = 8'h91;          // 无
                        3'd1: eff_code = 8'h92;          // 淡
                        3'd2: eff_code = 8'h95;          // 滑
                        3'd3: eff_code = 8'h98;          // 栅
                        3'd5: eff_code = 8'h9B;          // 粒
                        default: eff_code = 8'h96;       // 擦
                    endcase
                end
                3'd6: begin
                    case (e)
                        3'd0: eff_code = 8'h20;
                        3'd1: eff_code = 8'h93;          // 入
                        3'd2: eff_code = 8'h88;          // 动
                        3'd3: eff_code = 8'h99;          // 格
                        3'd5: eff_code = 8'h9C;          // 子
                        default: eff_code = 8'h97;       // 除
                    endcase
                end
                3'd7: begin
                    case (e)
                        3'd1: eff_code = 8'h92;          // 淡
                        3'd3: eff_code = 8'h9A;          // 化
                        default: eff_code = 8'h20;
                    endcase
                end
                default: eff_code = (e == 3'd1) ? 8'h94  // 出（仅"淡入淡出"有第 8 字）
                                              : 8'h20;
            endcase
        end
    endfunction

    // ---------------- 粒子特效 OSD 码表（单行："粒子特效:无/雪花/星空"） ----------------
    // 粒0x9B 子0x9C（特0x8F 效0x90 无0x91 雪0x9D 花0x9E 星0xA1 空0xA2 复用）
    function [7:0] snow_code;
        input [3:0] ci;
        input [1:0] lv;                     // 0=无 2=雪花 3=星空
        begin
            case (ci)
                3'd0: snow_code = 8'h9B;                  // 粒
                3'd1: snow_code = 8'h9C;                  // 子
                3'd2: snow_code = 8'h8F;                  // 特
                3'd3: snow_code = 8'h90;                  // 效
                3'd4: snow_code = 8'h3A;                  // ':'
                3'd5: begin
                    case (lv)
                        2'd0: snow_code = 8'h91;          // 无
                        2'd3: snow_code = 8'hA1;          // 星
                        default: snow_code = 8'h9D;       // 雪
                    endcase
                end
                3'd6: begin
                    case (lv)
                        2'd0: snow_code = 8'h20;
                        2'd3: snow_code = 8'hA2;          // 空
                        default: snow_code = 8'h9E;       // 花
                    endcase
                end
                default: snow_code = 8'h20;
            endcase
        end
    endfunction

    // ---------------- SW4 音频可视化 OSD 码表（单行 8 单元："音频可视化已显示/已隐藏"） ----------------
    // 音0x84 化0x9A 复用；频0xA8 可0xA9 视0xAA 已0xAB 显0xAC 示0xAD 隐0xAE 藏0xAF
    function [7:0] viz_code;
        input [3:0] ci;             // 4 位：与调用方一致，防未来加宽时再踩截断坑
        input       on;             // 1=已显示 0=已隐藏
        begin
            case (ci)
                3'd0: viz_code = 8'h84;                  // 音
                3'd1: viz_code = 8'hA8;                  // 频
                3'd2: viz_code = 8'hA9;                  // 可
                3'd3: viz_code = 8'hAA;                  // 视
                3'd4: viz_code = 8'h9A;                  // 化
                3'd5: viz_code = 8'hAB;                  // 已
                3'd6: viz_code = on ? 8'hAC : 8'hAE;     // 显 / 隐
                3'd7: viz_code = on ? 8'hAD : 8'hAF;     // 示 / 藏
                default: viz_code = 8'h20;
            endcase
        end
    endfunction

    // ---------------- 分辨率 OSD 码表（两行 8 单元：上行"分辨率"居中，下行数字） ----------------
    // 分0xA3 辨0xA4 率0xA5（数字/x 为 ASCII）
    function [7:0] res_code;
        input [3:0] ci;
        input [1:0] t;
        input       rw;             // 0=上行标签 1=下行数字
        begin
            if (!rw) begin          // "分辨率"居中于 8 单元 → 单元 2..4
                case (ci)
                    4'd2: res_code = 8'hA3;          // 分
                    4'd3: res_code = 8'hA4;          // 辨
                    4'd4: res_code = 8'hA5;          // 率
                    default: res_code = 8'h20;
                endcase
            end
            else begin
                case (t)
                    2'd0: case (ci)                  // 640x480
                        4'd0: res_code = 8'h36;  4'd1: res_code = 8'h34;
                        4'd2: res_code = 8'h30;  4'd3: res_code = 8'h78;  // 'x'
                        4'd4: res_code = 8'h34;  4'd5: res_code = 8'h38;
                        4'd6: res_code = 8'h30;  default: res_code = 8'h20;
                    endcase
                    2'd1: case (ci)                  // 1024x768
                        4'd0: res_code = 8'h31;  4'd1: res_code = 8'h30;
                        4'd2: res_code = 8'h32;  4'd3: res_code = 8'h34;
                        4'd4: res_code = 8'h78;  4'd5: res_code = 8'h37;
                        4'd6: res_code = 8'h36;  default: res_code = 8'h38;
                    endcase
                    default: case (ci)               // 1280x960
                        4'd0: res_code = 8'h31;  4'd1: res_code = 8'h32;
                        4'd2: res_code = 8'h38;  4'd3: res_code = 8'h30;
                        4'd4: res_code = 8'h78;  4'd5: res_code = 8'h39;
                        4'd6: res_code = 8'h36;  default: res_code = 8'h30;
                    endcase
                endcase
            end
        end
    endfunction

    // ---------------- OSD 区域几何：三独立通道，按区域空间合并 ----------------
    // 菜单通道（左）：菜单 x 8..151（9 单元）y 8..151（7 行）；SW1 单行 x 8..135 y 8..31
    // 分辨率通道（右上）：x 504..632（8 单元）两行 y 8..51，上行"分辨率"下行数字
    // SW4 通道（顶部中段）：x 168..295（8 单元）单行 y 8..31，"音频可视化已显示/已隐藏"
    // 三通道 x 范围不重叠，逐像素只可能命中其一
    wire        in_box_m = menu_mode
                         ? ((x_cnt >= 11'd8) && (x_cnt < 11'd136) &&
                            (y_cnt >= 11'd8) && (y_cnt < 11'd32))
                         : ((x_cnt >= 11'd8) && (x_cnt < 11'd152) &&
                            (y_cnt >= 11'd8) && (y_cnt < 11'd152));
    wire        in_box_r = (x_cnt >= 11'd504) && (x_cnt < 11'd632) &&
                           (y_cnt >= 11'd8) && (y_cnt < 11'd52);

    wire [2:0]  ln     = (y_cnt < 11'd28)  ? 3'd0 :
                         (y_cnt < 11'd48)  ? 3'd1 :
                         (y_cnt < 11'd68)  ? 3'd2 :
                         (y_cnt < 11'd88)  ? 3'd3 :
                         (y_cnt < 11'd108) ? 3'd4 :
                         (y_cnt < 11'd128) ? 3'd5 : 3'd6;
    wire        res_row = (y_cnt >= 11'd32);                 // 分辨率 OSD 下行
    wire [3:0]  trow_m = menu_mode
                       ? (y_cnt - 11'd12)
                       : ((y_cnt < 11'd28)  ? (y_cnt - 11'd12) :
                          (y_cnt < 11'd48)  ? (y_cnt - 11'd32) :
                          (y_cnt < 11'd68)  ? (y_cnt - 11'd52) :
                          (y_cnt < 11'd88)  ? (y_cnt - 11'd72) :
                          (y_cnt < 11'd108) ? (y_cnt - 11'd92) :
                          (y_cnt < 11'd128) ? (y_cnt - 11'd112) : (y_cnt - 11'd132));
    wire [3:0]  trow_r = res_row ? (y_cnt - 11'd32) : (y_cnt - 11'd12);
    wire        in_txt_m = menu_mode
                         ? ((y_cnt >= 11'd12) && (y_cnt < 11'd28))
                         : ((y_cnt >= 11'd12) && (y_cnt < 11'd28) ||
                            (y_cnt >= 11'd32) && (y_cnt < 11'd48) ||
                            (y_cnt >= 11'd52) && (y_cnt < 11'd68) ||
                            (y_cnt >= 11'd72) && (y_cnt < 11'd88) ||
                            (y_cnt >= 11'd92) && (y_cnt < 11'd108) ||
                            (y_cnt >= 11'd112) && (y_cnt < 11'd128) ||
                            (y_cnt >= 11'd132) && (y_cnt < 11'd148));
    wire        in_txt_r = ((y_cnt >= 11'd12) && (y_cnt < 11'd28)) ||
                           ((y_cnt >= 11'd32) && (y_cnt < 11'd48));

    wire [10:0] xrel_m  = x_cnt - 11'd8;
    wire [10:0] xrel_r  = x_cnt - 11'd504;
    wire [3:0]  ci_m    = xrel_m[7:4];                        // 单元 0..8（4 位，防截断）
    wire [3:0]  col_m   = xrel_m[3:0];
    wire [3:0]  ci_r    = xrel_r[7:4];
    wire [3:0]  col_r   = xrel_r[3:0];
    wire        in_box_v = (x_cnt >= 11'd168) && (x_cnt < 11'd296) &&
                           (y_cnt >= 11'd8)  && (y_cnt < 11'd32);
    wire        in_txt_v = (y_cnt >= 11'd12) && (y_cnt < 11'd28);
    wire [10:0] xrel_v   = x_cnt - 11'd168;
    wire [3:0]  ci_v     = xrel_v[7:4];
    wire [3:0]  col_v    = xrel_v[3:0];
    wire [3:0]  trow_v   = y_cnt - 11'd12;
    wire [7:0]  code_m  = menu_mode ? sw1_code(ci_m, sw1_s1)
                        : (ln == 3'd6) ? snow_code(ci_m, snow_lv)          // 第 7 行：粒子特效
                        : (ln == 3'd5) ? eff_code(ci_m, eff_lvl)           // 第 6 行：转场特效
                                             : osd_code(ci_m, ln, con_lv, bri_lv, vol_lv, rat_lv, sat_lv);
    wire [7:0]  code_r  = res_code(ci_r, res_lv, res_row);
    wire [7:0]  code_v  = viz_code(ci_v, viz_on);

    // 通道激活（含各自显示标志）→ 合并为单路流水
    wire        act_m   = in_box_m && show_menu;
    wire        act_r   = in_box_r && show_res;
    wire        act_v   = in_box_v && show_viz;
    wire        in_box  = act_m | act_r | act_v;
    wire        in_txt  = (act_m & in_txt_m) | (act_r & in_txt_r) | (act_v & in_txt_v);
    wire [7:0]  code    = act_m ? code_m : (act_r ? code_r : code_v);
    wire [3:0]  col     = act_m ? col_m : (act_r ? col_r : col_v);
    wire [3:0]  trow    = act_m ? trow_m : (act_r ? trow_r : trow_v);
    wire        hl_w    = act_m ? (~menu_mode && (ln == menu_sel)) : 1'b1;   // 分辨率/SW4 行恒白字

    // ASCII 用 font8x16_rom；汉字用 osd_cn_rom（码 0x80+序号，38 字 ROM，序号 6 位）
    wire [10:0] ascii_addr = ((code - 8'd32) * 8'd16) + {7'd0, trow};
    wire [9:0]  cn_addr    = {code[5:0], trow};          // 序号*16+行

    // ---------------- 1 拍流水（对齐字体 ROM 同步读出） ----------------
    reg        d_de, d_vs, d_in_box, d_in_txt;
    reg [23:0] d_rgb;
    reg [3:0]  d_col;
    reg [7:0]  d_code;
    reg        d_hl;                                     // 本行高亮（打一拍）
    reg [10:0] ascii_addr_r;
    reg [9:0]  cn_addr_r;

    wire [7:0]  font_bits;
    wire [15:0] cn_bits;

    font8x16_rom u_font(
        .clk  (I_clk),
        .addr (ascii_addr_r),
        .dout (font_bits)
    );

    osd_cn_rom u_cn(
        .clk  (I_clk),
        .addr (cn_addr_r),
        .dout (cn_bits)
    );

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            d_de     <= 1'b0;
            d_vs     <= 1'b0;
            d_in_box <= 1'b0;
            d_in_txt <= 1'b0;
            d_rgb    <= 24'd0;
            d_col    <= 4'd0;
            d_code   <= 8'd0;
            d_hl     <= 1'b0;
            ascii_addr_r <= 11'd0;
            cn_addr_r    <= 10'd0;
        end
        else begin
            d_de     <= I_de;
            d_vs     <= I_vs;
            d_in_box <= in_box;
            d_in_txt <= in_txt;
            d_rgb    <= I_rgb;
            d_col    <= col;
            d_code   <= code;
            d_hl     <= hl_w;  // 菜单行按 K3 计数高亮；SW1/分辨率行恒白字
            ascii_addr_r <= ascii_addr;
            cn_addr_r    <= cn_addr;
        end
    end

    // ---------------- 亮度缩放 + 对比度缩放（组合） ----------------
    wire [16:0] r_p = {8'd0, d_rgb[23:16]} * gain;
    wire [16:0] g_p = {8'd0, d_rgb[15:8]}  * gain;
    wire [16:0] b_p = {8'd0, d_rgb[7:0]}   * gain;
    wire [7:0]  r_b = (r_p[16:8] > 8'd255) ? 8'd255 : r_p[16:8];
    wire [7:0]  g_b = (g_p[16:8] > 8'd255) ? 8'd255 : g_p[16:8];
    wire [7:0]  b_b = (b_p[16:8] > 8'd255) ? 8'd255 : b_p[16:8];

    // 对比度：out = clamp(((rgb-128)*c)>>8 + 128)
    wire signed [9:0]  r_d = {2'b00, r_b} - 10'sd128;
    wire signed [9:0]  g_d = {2'b00, g_b} - 10'sd128;
    wire signed [9:0]  b_d = {2'b00, b_b} - 10'sd128;
    wire signed [19:0] r_c = r_d * $signed({1'b0, cgain});
    wire signed [19:0] g_c = g_d * $signed({1'b0, cgain});
    wire signed [19:0] b_c = b_d * $signed({1'b0, cgain});
    wire signed [11:0] r_s = (r_c >>> 8) + 12'sd128;
    wire signed [11:0] g_s = (g_c >>> 8) + 12'sd128;
    wire signed [11:0] b_s = (b_c >>> 8) + 12'sd128;
    wire [7:0] r_f = (r_s > 12'sd255) ? 8'd255 : ((r_s < 12'sd0) ? 8'd0 : r_s[7:0]);
    wire [7:0] g_f = (g_s > 12'sd255) ? 8'd255 : ((g_s < 12'sd0) ? 8'd0 : g_s[7:0]);
    wire [7:0] b_f = (b_s > 12'sd255) ? 8'd255 : ((b_s < 12'sd0) ? 8'd0 : b_s[7:0]);

    // ---------------- 饱和度（亮度加权法，作用于亮度/对比度之后、OSD 之下） ----------------
    // Y = (77R+150G+29B)>>8；C' = Y + ((C-Y)*gain)>>8；gain 0=纯黑白 256=原样 512=翻倍
    wire [15:0] lum_acc = r_f * 8'd77 + g_f * 8'd150 + b_f * 8'd29;
    wire [7:0]  lum     = lum_acc[15:8];

    wire signed [8:0] rd = $signed({1'b0, r_f}) - $signed({1'b0, lum});
    wire signed [8:0] gd = $signed({1'b0, g_f}) - $signed({1'b0, lum});
    wire signed [8:0] bd = $signed({1'b0, b_f}) - $signed({1'b0, lum});

    wire signed [18:0] rp = rd * $signed({1'b0, sgain});
    wire signed [18:0] gp = gd * $signed({1'b0, sgain});
    wire signed [18:0] bp = bd * $signed({1'b0, sgain});

    wire signed [10:0] r_sum = $signed({3'b0, lum}) + (rp >>> 8);
    wire signed [10:0] g_sum = $signed({3'b0, lum}) + (gp >>> 8);
    wire signed [10:0] b_sum = $signed({3'b0, lum}) + (bp >>> 8);

    wire [7:0] r_o = (r_sum < 0) ? 8'd0 : ((r_sum > 11'sd255) ? 8'd255 : r_sum[7:0]);
    wire [7:0] g_o = (g_sum < 0) ? 8'd0 : ((g_sum > 11'sd255) ? 8'd255 : g_sum[7:0]);
    wire [7:0] b_o = (b_sum < 0) ? 8'd0 : ((b_sum > 11'sd255) ? 8'd255 : b_sum[7:0]);

    // ---------------- 输出 ----------------
    assign O_vs = d_vs;
    assign O_de = d_de;

    always @(*) begin
        if (d_in_box) begin
            // OSD（双通道已在 in_box 合并时含各自显示标志）：
            // 高亮行白字亮底，其余灰字暗底（均不受画面亮度影响）
            if (d_in_txt && d_code[7] && cn_bits[4'd15 - d_col])
                O_rgb = d_hl ? 24'hFFFFFF : 24'h909090;
            else if (d_in_txt && !d_code[7] && (d_col >= 4'd4) && (d_col <= 4'd11)
                     && font_bits[4'd11 - d_col])
                O_rgb = d_hl ? 24'hFFFFFF : 24'h909090;
            else
                O_rgb = d_hl ? 24'h383838 : 24'h1A1A1A;
        end
        else begin
            O_rgb = {r_o, g_o, b_o};        // 亮度+对比度+饱和度缩放后的画面
        end
    end

endmodule
