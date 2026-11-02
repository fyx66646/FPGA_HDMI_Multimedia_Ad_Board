// ============================================================================
// text_ram_dp : 字幕文字双时钟 RAM（256 x 8bit）
// 写侧 sd_card_clk（wav_reader 抓取音频头里的字幕时写入，文字从地址 0 存起），
// 读侧 video_clk（subtitle_overlay 逐像素取字符，读地址 = 画布字符序号 - 80）。
// 写只在切轨瞬间发生，读侧越界/空隙地址读到的内容被渲染端屏蔽，无冲突。
// ============================================================================
module text_ram_dp (
    input  wire       clkw,
    input  wire       we,
    input  wire [7:0] waddr,
    input  wire [7:0] wdata,

    input  wire       clkr,
    input  wire [7:0] raddr,
    output reg  [7:0] dout
);

    reg [7:0] mem [0:255];

    always @(posedge clkw) begin
        if (we)
            mem[waddr] <= wdata;
    end

    always @(posedge clkr) begin
        dout <= mem[raddr];
    end

endmodule
