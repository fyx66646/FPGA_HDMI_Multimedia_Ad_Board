// ============================================================================
// sd_read_arbiter : SD 卡扇区读仲裁器
// 内部独占一个 sd_card_top（SPI 物理控制器），对上提供两个"扇区读主口"：
//   主口 IMG —— 图片链路（sd_card_bmp / bmp_read），协议与原 sd_card_top 一致
//   主口 AUD —— 音频链路（sd_wav_reader）
// 仲裁策略（扇区粒度，仅在两个主口都无在途扇区、且当前主口撤销请求的
// 空闲窗口里切换，绝不打断进行中的扇区读）：
//   1. 音频欠载（aud_urgent=1，FIFO 低于低水位）→ 无条件优先音频；
//   2. 仅一方请求 → 授权该方；
//   3. 双方同时请求（如两条链路都在顺序扫描）→ 轮流，各拿约 50% 带宽。
// 带宽预算：SPI 25MHz 约 300us/扇区（~3300 扇区/s）；音频 48k/16bit/立体声
// 仅需 375 扇区/s，图片扫描/加载与音频播放可同时进行，互不卡顿。
// ============================================================================
module sd_read_arbiter (
    input  wire        clk,               // sd_card_clk 100MHz
    input  wire        rst,               // 高有效

    // 主口 IMG：图片链路
    input  wire        img_sd_sec_read,
    input  wire [31:0] img_sd_sec_read_addr,
    output wire [7:0]  img_sd_sec_read_data,
    output wire        img_sd_sec_read_data_valid,
    output wire        img_sd_sec_read_end,

    // 主口 AUD：音频链路
    input  wire        aud_sd_sec_read,
    input  wire [31:0] aud_sd_sec_read_addr,
    input  wire        aud_urgent,        // 音频 FIFO 低于低水位，需要插队
    output wire [7:0]  aud_sd_sec_read_data,
    output wire        aud_sd_sec_read_data_valid,
    output wire        aud_sd_sec_read_end,

    output wire        sd_init_done,

    // 调试直播（透传 sd_card_sec_read_write）
    output wire [4:0]  dbg_state,     // 控制器状态机（数码管 位4/位3 hex）
    output wire [5:0]  dbg_retry,     // 读失败计数（数码管 位2/位1 hex）

    // SPI 物理引脚
    output wire        SD_nCS,
    output wire        SD_DCLK,
    output wire        SD_MOSI,
    input  wire        SD_MISO
);

    localparam GRANT_IMG = 1'b0;
    localparam GRANT_AUD = 1'b1;

    reg grant;         // 当前授权的主口
    reg op_active;     // 有一个扇区读在途（从控制器接受请求到 read_end）

    wire [7:0] ctrl_data;
    wire       ctrl_valid;
    wire       ctrl_end;

    wire        mux_req;
    wire [31:0] mux_addr;

    // ---------------- sd_card_top 唯一实例 ----------------
    sd_card_top sd_card_top_m0(
        .clk                    (clk),
        .rst                    (rst),
        .SD_nCS                 (SD_nCS),
        .SD_DCLK                (SD_DCLK),
        .SD_MOSI                (SD_MOSI),
        .SD_MISO                (SD_MISO),
        .sd_init_done           (sd_init_done),
        .dbg_state              (dbg_state),
        .dbg_retry              (dbg_retry),
        .sd_sec_read            (mux_req),
        .sd_sec_read_addr       (mux_addr),
        .sd_sec_read_data       (ctrl_data),
        .sd_sec_read_data_valid (ctrl_valid),
        .sd_sec_read_end        (ctrl_end),
        .sd_sec_write           (1'b0),
        .sd_sec_write_addr      (32'd0),
        .sd_sec_write_data      (),
        .sd_sec_write_data_req  (),
        .sd_sec_write_end       ()
    );

    assign mux_req  = grant ? aud_sd_sec_read      : img_sd_sec_read;
    assign mux_addr = grant ? aud_sd_sec_read_addr : img_sd_sec_read_addr;

    // 控制器输出只路由给当前授权的主口（授权在一个扇区读期间保持不变）
    assign img_sd_sec_read_data       = ctrl_data;
    assign img_sd_sec_read_data_valid = ctrl_valid && (grant == GRANT_IMG);
    assign img_sd_sec_read_end        = ctrl_end   && (grant == GRANT_IMG);

    assign aud_sd_sec_read_data       = ctrl_data;
    assign aud_sd_sec_read_data_valid = ctrl_valid && (grant == GRANT_AUD);
    assign aud_sd_sec_read_end        = ctrl_end   && (grant == GRANT_AUD);

    // ---------------- 仲裁 ----------------
    // op_active：控制器接受请求（S_WAIT_READ_WRITE 看到 mux_req）置位，
    //            对应扇区 read_end 脉冲清零。CMD17 至少数百微秒，不会一拍完成。
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            grant     <= GRANT_IMG;
            op_active <= 1'b0;
        end
        else begin
            if (op_active) begin
                if (ctrl_end)
                    op_active <= 1'b0;
            end
            else if (mux_req) begin
                op_active <= 1'b1;              // 控制器已带上新请求，锁定授权
            end
            else begin
                // 空闲窗口：当前主口没在请求，可以安全切换
                if (aud_urgent && aud_sd_sec_read)
                    grant <= GRANT_AUD;          // 音频欠载，插队
                else if (!img_sd_sec_read && aud_sd_sec_read)
                    grant <= GRANT_AUD;
                else if (img_sd_sec_read && !aud_sd_sec_read)
                    grant <= GRANT_IMG;
                else if (img_sd_sec_read && aud_sd_sec_read)
                    grant <= ~grant;             // 都在扫描：轮流
            end
        end
    end

endmodule
