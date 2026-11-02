// ============================================================================
// font_cn_ram : 汉字子集点阵双时钟 RAM（2048 x 16bit = 128 字 x 16 行）
// 写侧 sd_card_clk（wav_reader 逐扇区流式灌入字库数据），读侧 video_clk
// （subtitle_overlay 逐像素取行点阵）。地址 = (code-0x80)*16 + row，
// 每行 16bit，bit15 = 最左像素。写只在切轨瞬间发生，读侧无效数据被屏蔽。
// ============================================================================
module font_cn_ram (
    input  wire        clkw,
    input  wire        we,
    input  wire [10:0] waddr,
    input  wire [15:0] wdata,

    input  wire        clkr,
    input  wire [10:0] raddr,
    output reg  [15:0] dout
);

    reg [15:0] mem [0:2047];

    always @(posedge clkw) begin
        if (we)
            mem[waddr] <= wdata;
    end

    always @(posedge clkr) begin
        dout <= mem[raddr];
    end

endmodule
