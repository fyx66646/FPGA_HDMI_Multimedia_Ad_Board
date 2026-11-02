// ============================================================================
// subtitle_overlay : 底部滚动字幕渲染（中英混排 16px 单元版，video_clk 域）
// 在 640x480 图像的第 452..475 行叠加 24px 高字幕条，白字、深色底。
// 每单元 16px：
//   汉字 (0x80..0xFF) → font_cn_ram 16x16 点阵（bit15=最左像素）
//   ASCII(0x20..0x7E) → 片内 8x16 点阵，水平居中在单元第 4..11 列
// 跑马灯：画布 = [640px 入场空隙][文字]，文字从右侧滑入、滚出左侧
// 后重新入场，每帧 1 像素（约 60px/s）。
//
// 数据通路：RGB/DE/VS 统一延迟 4 拍（文字 RAM 一拍 + 字库 ROM/RAM
// 一拍的同步读出延迟），无字幕时纯直通。文字行限制在 456..471，
// 上下各 4px 留边不画点。
// ============================================================================
module subtitle_overlay (
    input  wire        I_clk,          // video_clk 25MHz
    input  wire        I_rst,          // 高有效

    input  wire        I_vs,
    input  wire        I_de,
    input  wire [23:0] I_rgb,

    // 文字 RAM 读口（text_ram_dp 读侧，模块外部例化）
    output reg  [7:0]  O_ram_raddr,
    input  wire [7:0]  I_ram_dout,

    // 汉字字库读口（font_cn_ram 读侧，模块外部例化）
    // 地址组合输出（与 font8x16_rom 一样一拍读出），bit15=最左像素
    output wire [10:0] O_cn_raddr,
    input  wire [15:0] I_cn_dout,

    // 字幕内容跨域握手（来自 sd_card_clk 域 wav_reader，准静态信号）
    input  wire        I_txt_toggle,   // 每次切轨抓取完文字+字库后翻转一次
    input  wire [7:0]  I_txt_len,      // 单元数 0..100（写入 RAM 后保持稳定）

    output reg         O_vs,
    output reg         O_de,
    output reg  [23:0] O_rgb
);

    localparam [23:0] BAR_COLOR  = 24'h101010;   // 字幕条底色
    localparam [23:0] TEXT_COLOR = 24'hF0F0F0;   // 文字颜色

    localparam [10:0] BAR_TOP = 11'd452;
    localparam [10:0] BAR_BOT = 11'd475;
    localparam [10:0] TXT_TOP = 11'd456;         // 4px 上留边后为 16 行文字
    localparam [10:0] TXT_BOT = 11'd471;

    localparam [7:0] GAP_CELLS = 8'd40;          // 640px 入场空隙 = 40 单元

    // ---------------- 帧 / 行 / 像素计数 ----------------
    reg        vs_d, de_d;
    reg [10:0] y_cnt, x_cnt;
    reg [11:0] scroll_off;
    reg [11:0] off_limit_r;                      // 640+16*len（在下方锁存块中更新）

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            vs_d       <= 1'b0;
            de_d       <= 1'b0;
            y_cnt      <= 11'd0;
            x_cnt      <= 11'd0;
            scroll_off <= 12'd0;
        end
        else begin
            vs_d <= I_vs;
            de_d <= I_de;

            if (vs_d != I_vs) begin              // VS 跳变 = 新帧
                y_cnt <= 11'd0;
                if (scroll_off >= off_limit_r)   // 滚出左侧后从头入场（右侧）
                    scroll_off <= 12'd0;
                else
                    scroll_off <= scroll_off + 12'd1;   // 1px/帧 ≈ 60px/s
            end
            else if (!I_de && de_d) begin        // DE 下降 = 一行结束
                y_cnt <= y_cnt + 11'd1;
            end

            if (I_de)
                x_cnt <= (x_cnt == 11'd639) ? 11'd0 : (x_cnt + 11'd1);
            else
                x_cnt <= 11'd0;
        end
    end

    // ---------------- 字幕长度跨域锁存 ----------------
    reg [1:0] tgl_sync;
    reg       tgl_seen;
    reg [7:0] txt_len;

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            tgl_sync    <= 2'b00;
            tgl_seen    <= 1'b0;
            txt_len     <= 8'd0;
            off_limit_r <= 12'd640;
        end
        else begin
            tgl_sync <= {tgl_sync[0], I_txt_toggle};
            if (tgl_sync[1] != tgl_seen) begin
                tgl_seen    <= tgl_sync[1];
                txt_len     <= I_txt_len;        // 长度+字库在 toggle 前已写好并稳定
                off_limit_r <= 12'd640 + {4'b0000, I_txt_len, 4'b0000};  // 640+16*len
            end
        end
    end

    // ---------------- 像素流水线（统一 4 拍延迟）----------------
    // 画布 = [40 单元空隙][文字]，屏幕窗口 [off, off+640)，每单元 16px
    wire [11:0] scr_x       = scroll_off + {1'b0, x_cnt};           // 0..2878
    wire [7:0]  cell_idx    = scr_x[11:4];                          // /16 → 0..179
    wire [3:0]  col_in_cell = scr_x[3:0];                           // 单元内列 0=最左
    wire        in_bar      = (y_cnt >= BAR_TOP) && (y_cnt <= BAR_BOT);
    wire        in_txt_row  = (y_cnt >= TXT_TOP) && (y_cnt <= TXT_BOT);
    wire [10:0] row_full    = y_cnt - TXT_TOP;
    wire [3:0]  row_in_txt  = row_full[3:0];                        // 文字行 0..15
    wire [7:0]  ram_addr    = cell_idx - GAP_CELLS;                 // 文字在 RAM 从 0 存起

    // 时序：O_ram_raddr 于 d1 拍有效 → RAM 同步读出 code_r 于 d2 拍有效
    //       → 两个字库地址（组合）→ 点阵行数据于 d3 拍有效
    reg        d1_de, d1_vs, d1_in_bar, d1_txt_row;
    reg [23:0] d1_rgb;
    reg [3:0]  d1_col, d1_row;
    reg [7:0]  d1_cell;

    reg        d2_de, d2_vs, d2_in_bar, d2_txt_row;
    reg [23:0] d2_rgb;
    reg [3:0]  d2_col, d2_row;
    reg [7:0]  d2_cell;

    reg        d3_de, d3_vs, d3_in_bar, d3_txt_row;
    reg [23:0] d3_rgb;
    reg [3:0]  d3_col;
    reg [7:0]  d3_cell, d3_code;

    wire [7:0]  code_r = I_ram_dout;                          // d2 拍有效
    wire [6:0]  cidx   = code_r[6:0] - 8'd32;                 // (code-0x20)
    wire [10:0] rom_addr = (code_r >= 8'd32 && code_r <= 8'd126)
                           ? {cidx, d2_row}                   // (code-32)*16 + row
                           : 11'd0;
    wire [10:0] cn_addr = {code_r[6:0], d2_row};              // (code-0x80)*16 + row

    assign O_cn_raddr = cn_addr;

    wire [7:0]  font_bits;                                     // d3 拍有效
    wire [15:0] cn_bits = I_cn_dout;                           // d3 拍有效（font_cn_ram 读出）

    font8x16_rom u_font(
        .clk  (I_clk),
        .addr (rom_addr),
        .dout (font_bits)
    );

    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            O_ram_raddr <= 8'd0;
            d1_de <= 1'b0;  d1_vs <= 1'b0;  d1_in_bar <= 1'b0;  d1_txt_row <= 1'b0;
            d1_rgb <= 24'd0; d1_col <= 4'd0; d1_row <= 4'd0; d1_cell <= 8'd0;
            d2_de <= 1'b0;  d2_vs <= 1'b0;  d2_in_bar <= 1'b0;  d2_txt_row <= 1'b0;
            d2_rgb <= 24'd0; d2_col <= 4'd0; d2_row <= 4'd0; d2_cell <= 8'd0;
            d3_de <= 1'b0;  d3_vs <= 1'b0;  d3_in_bar <= 1'b0;  d3_txt_row <= 1'b0;
            d3_rgb <= 24'd0; d3_col <= 4'd0; d3_cell <= 8'd0; d3_code <= 8'd0;
            O_vs <= 1'b0;  O_de <= 1'b0;  O_rgb <= 24'd0;
        end
        else begin
            // stage0 → stage1
            O_ram_raddr <= ram_addr;
            d1_de       <= I_de;
            d1_vs       <= I_vs;
            d1_in_bar   <= in_bar;
            d1_txt_row  <= in_bar && in_txt_row;
            d1_rgb      <= I_rgb;
            d1_col      <= col_in_cell;
            d1_row      <= row_in_txt;
            d1_cell     <= cell_idx;

            // stage1 → stage2
            d2_de       <= d1_de;
            d2_vs       <= d1_vs;
            d2_in_bar   <= d1_in_bar;
            d2_txt_row  <= d1_txt_row;
            d2_rgb      <= d1_rgb;
            d2_col      <= d1_col;
            d2_row      <= d1_row;
            d2_cell     <= d1_cell;

            // stage2 → stage3（code_r 于 d2 拍读出，随流水对齐到 d3）
            d3_de       <= d2_de;
            d3_vs       <= d2_vs;
            d3_in_bar   <= d2_in_bar;
            d3_txt_row  <= d2_txt_row;
            d3_rgb      <= d2_rgb;
            d3_col      <= d2_col;
            d3_cell     <= d2_cell;
            d3_code     <= code_r;

            // 输出（第 4 拍）：font_bits / cn_bits 与 d3_* 同拍有效
            // 文字区域：画布单元 40 .. 40+len-1（入场空隙之后）
            // ASCII 8px 字形居中：有效列 4..11，取 font_bits[11-col]
            // 汉字 16px：bit15=最左，取 cn_bits[15-col]
            O_vs <= d3_vs;
            O_de <= d3_de;
            if (d3_de && d3_in_bar && d3_txt_row && (txt_len != 8'd0)
                && (d3_cell >= GAP_CELLS)
                && ((d3_cell - GAP_CELLS) < txt_len)
                && (((d3_code >= 8'd32) && (d3_code <= 8'd126)
                     && (d3_col >= 4'd4) && (d3_col <= 4'd11)
                     && font_bits[4'd11 - d3_col])
                    || (d3_code[7] && cn_bits[4'd15 - d3_col])))
                O_rgb <= TEXT_COLOR;
            else if (d3_de && d3_in_bar && (txt_len != 8'd0))
                O_rgb <= BAR_COLOR;
            else
                O_rgb <= d3_rgb;
        end
    end

endmodule
