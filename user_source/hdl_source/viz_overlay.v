// ============================================================================
// viz_overlay : 底部 32 根离散频谱条形（video_clk 域，纯 FFT 直出）
// 屏幕底部（字幕条上方）32 根条形：每根 18px 宽 + 2px 缝隙（共 20px/频段），
// 高度 = 该频段电平 x 4px（6bit 电平 0..63 → 0..252px），无包络无平滑。
// 频率从左（低频）到右（高频）排列，条体由密集颜色点构成（2x2 网格每格 1 点），
// 32 频段独立彩虹配色。列→频段号/条内位置的映射在 viz_table_rom 查找表。
// 视频通路统一延迟 1 拍（对齐 ROM 同步读出），vs/de/rgb 同时延，无感知。
// SW4 开关（I_show，2026-10-28）：低=隐藏频谱条（直通原画面），高=显示。
// ============================================================================
module viz_overlay (
    input  wire        I_clk,          // video_clk 25MHz
    input  wire        I_rst,          // 高有效

    input  wire        I_vs,
    input  wire        I_de,
    input  wire [23:0] I_rgb,

    input  wire [191:0] I_levels,      // 32 频段 x 6bit（来自 spectrum_core）
    input  wire        I_show,         // SW4：1=显示频谱条，0=隐藏（同步后电平）

    output wire        O_vs,
    output wire        O_de,
    output reg  [23:0] O_rgb
);

    localparam [10:0] BASELINE = 11'd448;   // 频谱底边（字幕条 452 之上）

    // ---------------- 帧 / 行 / 像素计数（与输入像素同拍） ----------------
    reg        vs_d, de_d;
    reg [10:0] y_cnt, x_cnt;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            vs_d  <= 1'b0;
            de_d  <= 1'b0;
            y_cnt <= 11'd0;
            x_cnt <= 11'd0;
        end
        else begin
            vs_d <= I_vs;
            de_d <= I_de;

            if (vs_d != I_vs)
                y_cnt <= 11'd0;
            else if (!I_de && de_d)
                y_cnt <= y_cnt + 11'd1;

            if (I_de)
                x_cnt <= (x_cnt == 11'd639) ? 11'd0 : (x_cnt + 11'd1);
            else
                x_cnt <= 11'd0;
        end
    end

    // ---------------- 1 拍延迟通路 + ROM 同步读 ----------------
    reg        d_de, d_vs;
    reg [23:0] d_rgb;
    reg [10:0] d_x, d_y;

    wire [9:0] rom_dout;   // 实例输出必须接 net（rom 内部才是 reg）

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            d_de  <= 1'b0;
            d_vs  <= 1'b0;
            d_rgb <= 24'd0;
            d_x   <= 11'd0;
            d_y   <= 11'd0;
        end
        else begin
            d_de  <= I_de;
            d_vs  <= I_vs;
            d_rgb <= I_rgb;
            d_x   <= x_cnt;
            d_y   <= y_cnt;
        end
    end

    viz_table_rom u_table(
        .clk  (I_clk),
        .addr (x_cnt),
        .dout (rom_dout)         // 1 拍后有效，与 d_*（同像素）对齐
    );

    wire [4:0] band       = rom_dout[9:5];   // 频段号 0..31
    wire [4:0] col_in_bar = rom_dout[4:0];   // 条内列位置 0..19（18..19 为 2px 缝隙）

    // ---------------- 频段电平（32 频段 x 6bit） ----------------
    function [5:0] lv_of;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  lv_of = I_levels[5:0];
                5'd1:  lv_of = I_levels[11:6];
                5'd2:  lv_of = I_levels[17:12];
                5'd3:  lv_of = I_levels[23:18];
                5'd4:  lv_of = I_levels[29:24];
                5'd5:  lv_of = I_levels[35:30];
                5'd6:  lv_of = I_levels[41:36];
                5'd7:  lv_of = I_levels[47:42];
                5'd8:  lv_of = I_levels[53:48];
                5'd9:  lv_of = I_levels[59:54];
                5'd10: lv_of = I_levels[65:60];
                5'd11: lv_of = I_levels[71:66];
                5'd12: lv_of = I_levels[77:72];
                5'd13: lv_of = I_levels[83:78];
                5'd14: lv_of = I_levels[89:84];
                5'd15: lv_of = I_levels[95:90];
                5'd16: lv_of = I_levels[101:96];
                5'd17: lv_of = I_levels[107:102];
                5'd18: lv_of = I_levels[113:108];
                5'd19: lv_of = I_levels[119:114];
                5'd20: lv_of = I_levels[125:120];
                5'd21: lv_of = I_levels[131:126];
                5'd22: lv_of = I_levels[137:132];
                5'd23: lv_of = I_levels[143:138];
                5'd24: lv_of = I_levels[149:144];
                5'd25: lv_of = I_levels[155:150];
                5'd26: lv_of = I_levels[161:156];
                5'd27: lv_of = I_levels[167:162];
                5'd28: lv_of = I_levels[173:168];
                5'd29: lv_of = I_levels[179:174];
                5'd30: lv_of = I_levels[185:180];
                default: lv_of = I_levels[191:186];
            endcase
        end
    endfunction

    // ---------------- 配色（32 档彩虹渐变，每频段独立配色） ----------------
    function [23:0] band_color;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  band_color = 24'hFF1E1E;   // 红（750Hz）
                5'd1:  band_color = 24'hFF3000;
                5'd2:  band_color = 24'hFF4A00;
                5'd3:  band_color = 24'hFF6500;
                5'd4:  band_color = 24'hFF7F00;
                5'd5:  band_color = 24'hFF9A00;
                5'd6:  band_color = 24'hFFB400;
                5'd7:  band_color = 24'hFFCF00;
                5'd8:  band_color = 24'hFFE900;
                5'd9:  band_color = 24'hE8F400;
                5'd10: band_color = 24'hC4F000;
                5'd11: band_color = 24'hA0EB00;
                5'd12: band_color = 24'h7CE700;
                5'd13: band_color = 24'h58E300;
                5'd14: band_color = 24'h34DF00;
                5'd15: band_color = 24'h00E030;
                5'd16: band_color = 24'h00E070;
                5'd17: band_color = 24'h00E0AF;
                5'd18: band_color = 24'h00E0E0;
                5'd19: band_color = 24'h00BFE8;
                5'd20: band_color = 24'h0098E8;
                5'd21: band_color = 24'h0071E8;
                5'd22: band_color = 24'h004AE8;
                5'd23: band_color = 24'h003CE8;
                5'd24: band_color = 24'h2040F0;
                5'd25: band_color = 24'h4850E8;
                5'd26: band_color = 24'h7060E0;
                5'd27: band_color = 24'h9870D8;
                5'd28: band_color = 24'hC080D0;
                5'd29: band_color = 24'hE890C8;
                5'd30: band_color = 24'hF0A0C0;
                default: band_color = 24'hF03CB0; // 品红（24kHz）
            endcase
        end
    endfunction

    // ---------------- 每根条高度与点阵判定（离散条形，无插值） ----------------
    wire [5:0]  lv_col  = lv_of(band);           // 该频段电平 0..63

    wire [8:0]  col_h  = {3'b0, lv_col} * 3'd4;   // 每级 4px，0..252px

    wire [10:0] y_top  = BASELINE - {3'b0, col_h};

    wire bar_on   = (col_in_bar < 5'd18);         // 条体 18px + 2px 缝隙
    wire in_field = d_de && (d_y <= BASELINE) && (d_y >= y_top);
    wire is_dot   = d_x[0] & d_y[0];              // 2x2 网格每格 1 点

    wire [23:0] dot_col = band_color(band);   // 32 频段独立配色

    // ---------------- 输出（与 d_* 同拍） ----------------
    assign O_vs = d_vs;
    assign O_de = d_de;

    always @(*) begin
        if (I_show && in_field && is_dot && bar_on)
            O_rgb = dot_col;
        else
            O_rgb = d_rgb;
    end

endmodule
