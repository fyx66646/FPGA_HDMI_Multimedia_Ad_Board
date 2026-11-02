// ============================================================================
// audio_fifo_dc : 双时钟异步 FIFO（写侧 sd_card_clk 100MHz，读侧 audio_mclk 12.288MHz）
// 用途：TF 卡音频数据缓冲。2048 x 16bit = 4KB ≈ 21ms @48kHz/16bit/立体声。
// 结构：格雷码指针 + 两级同步器，经典 Cummins 结构；
//       读侧为普通 FIFO（re 下一拍 dout/valid 有效）。
// 复位：异步高有效，两侧同时释放（rst_all）。
// ============================================================================
module audio_fifo_dc #(
    parameter DW = 16,                  // 数据位宽
    parameter AW = 11                   // 地址位宽，深度 = 2^AW = 2048
)(
    input  wire            rst,         // 异步复位，高有效

    // 写侧（sd_card_clk 域）
    input  wire            clkw,
    input  wire            we,
    input  wire [DW-1:0]   di,
    output wire            full,
    output wire [AW:0]     wrusedw,     // 写域视角的数据量（给 wav_reader 做水位）

    // 读侧（audio_mclk 域）
    input  wire            clkr,
    input  wire            re,
    output reg  [DW-1:0]   dout,
    output reg             valid_r,     // re 有效后一拍，dout 有效
    output wire            empty
);

    reg [DW-1:0] mem [0:(1<<AW)-1];

    // ---------------- 写域 ----------------
    reg  [AW:0] wbin;
    reg  [AW:0] wgray;
    reg  [AW:0] rgray_ws1, rgray_ws2;   // 读格雷码同步到写域
    reg  [AW:0] rbin_ws;                // 恢复的二进制读指针（组合逻辑）

    wire [AW:0] wbin_next  = wbin + {{AW{1'b0}},1'b1};
    wire [AW:0] wgray_next = (wbin_next >> 1) ^ wbin_next;
    wire        wfull      = (wgray_next == {~rgray_ws2[AW:AW-1], rgray_ws2[AW-2:0]});

    assign full = wfull;

    // 格雷码转二进制
    integer gi;
    always @(*) begin
        rbin_ws[AW] = rgray_ws2[AW];
        for (gi = AW-1; gi >= 0; gi = gi - 1)
            rbin_ws[gi] = rbin_ws[gi+1] ^ rgray_ws2[gi];
    end

    assign wrusedw = wbin - rbin_ws;

    always @(posedge clkw or posedge rst) begin
        if (rst) begin
            wbin      <= {(AW+1){1'b0}};
            wgray     <= {(AW+1){1'b0}};
            rgray_ws1 <= {(AW+1){1'b0}};
            rgray_ws2 <= {(AW+1){1'b0}};
        end else begin
            rgray_ws1 <= rgray;
            rgray_ws2 <= rgray_ws1;
            if (we && !wfull) begin
                mem[wbin[AW-1:0]] <= di;
                wbin  <= wbin_next;
                wgray <= wgray_next;
            end
        end
    end

    // ---------------- 读域 ----------------
    reg  [AW:0] rbin;
    reg  [AW:0] rgray;
    reg  [AW:0] wgray_rs1, wgray_rs2;   // 写格雷码同步到读域

    wire [AW:0] rbin_next  = rbin + {{AW{1'b0}},1'b1};
    wire [AW:0] rgray_next = (rbin_next >> 1) ^ rbin_next;
    wire        rempty     = (rgray == wgray_rs2);

    assign empty = rempty;

    wire do_read = re && !rempty;

    always @(posedge clkr or posedge rst) begin
        if (rst) begin
            rbin      <= {(AW+1){1'b0}};
            rgray     <= {(AW+1){1'b0}};
            wgray_rs1 <= {(AW+1){1'b0}};
            wgray_rs2 <= {(AW+1){1'b0}};
            dout      <= {DW{1'b0}};
            valid_r   <= 1'b0;
        end else begin
            wgray_rs1 <= wgray;
            wgray_rs2 <= wgray_rs1;
            valid_r   <= do_read;
            if (do_read) begin
                dout  <= mem[rbin[AW-1:0]];
                rbin  <= rbin_next;
                rgray <= rgray_next;
            end
        end
    end

endmodule
