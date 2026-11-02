
module sd_card_bmp #(
    parameter integer CLK_FREQ_HZ       = 100_000_000,
    parameter [31:0]  SCAN_START_SECTOR = 32'd0,
    parameter [31:0]  SCAN_MAX_SECTOR   = 32'd131071,
    parameter [2:0]   SCAN_TARGET_COUNT = 3'd4
)(
    input                       clk,
    input                       rst,
    input                       key_next,
    input                       auto_sw,        // SW1：高=自动轮播，低=手动轮播
    input                       eff_up_tgl,     // 转场特效 +1 事件（toggle，video 域 bri_sen_osd 发来）
    input                       eff_dn_tgl,     // 转场特效 -1 事件（toggle）
    output [2:0]                effect_sel,     // 当前特效编号：0无 1淡入淡出 2滑动 3栅格化 4线性擦除
    output reg                  fade_req,       // 请求视频域淡出（toggle，sd_card_clk 域）
    input                       fade_ack,       // 视频域淡出完成（toggle，video_clk 域）
    output [3:0]                state_code,
    output reg [2:0]            dbg_code,       // 诊断码：提交流程卡点定位（数码管显示）
    output [1:0]                disp_img_idx,   // 当前显示图片编号（供音频跟随切换）
    output reg                  img_commit_tgl, // 图片提交（切显示缓冲）toggle：触发分辨率 OSD
    output reg [1:0]            img_tier,       // 当前显示图的源分辨率挡 0/1/2（提交时锁存）
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,
    output reg                  display_valid,

    input                       write_finish_toggle,
    output reg [1:0]            write_buf_idx,
    output reg [1:0]            disp_buf_idx,

    output                      write_req,
    input                       write_req_ack,
    output                      write_en,
    output [31:0]               write_data,

    // SD 卡逻辑扇区读接口（经 sd_read_arbiter 与音频链路共享）
    input                       sd_init_done,
    output                      sd_sec_read,
    output [31:0]               sd_sec_read_addr,
    input [7:0]                 sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end
);

wire key_next_press;

// SW1 电平开关两级同步（异步输入 → clk 域）
reg auto_sw_s0, auto_sw_s1;

// 特效编号计数（bri_sen_osd 菜单第 5 行 K4 调整，toggle 事件 ±1，0..5 循环回绕）
reg [2:0] effect_cnt;

// 淡出握手：等视频域淡出完成后才提交新图
reg       wait_fade_ack;
reg [2:0] fack_sync;
wire      fade_ack_pulse;

// 特效 ±1 事件同步（toggle 电平边沿检测；不随 SD 软复位清零，避免假边沿）
reg [2:0] eup_sync;
reg [2:0] edn_sync;
wire      eff_up_pulse;
wire      eff_dn_pulse;

wire             bmp_data_wr_en;
wire [23:0]      bmp_data;
wire [1:0]       bmp_tier_w;      // bmp_read 头校验后的分辨率挡
wire             bmp_ready;
wire             scan_done;
wire             scan_found_valid;
wire [31:0]      scan_found_sector;
wire [2:0]       scan_found_total;

reg              scan_start_pulse;
reg              load_start_pulse;
reg [31:0]       load_sector;
reg              scan_kicked;
reg              first_image_committed;
reg              auto_play_en;
reg [31:0]       auto_cnt;
reg [2:0]        img_found_count;
reg [1:0]        img_idx;              // 当前真正显示中的图片编号
reg [1:0]        load_idx;             // 当前正在写入的图片编号
reg [1:0]        pending_buf_idx;      // 当前正在写入的目标缓冲区
reg [31:0]       img_sector0;
reg [31:0]       img_sector1;
reg [31:0]       img_sector2;
reg [31:0]       img_sector3;
reg              next_req_pending;
reg              load_busy;

// 加载失败恢复：看门狗（3 秒 @100MHz）与"未发起写"检测
reg              load_wr_seen;      // 本次加载期间是否发起过写请求
reg [31:0]       load_wd;           // 加载看门狗计数
reg              load_abort_pulse;  // 中止 bmp_read 当前加载

// bmp_ready 先表示“源文件读取/送 FIFO 完成”；真正切显示要等 write_finish_toggle 同步后
reg              source_done_seen;
reg              write_finish_seen;   // 本次加载是否收到过帧写完成（诊断用）
reg              sd_init_d0, sd_init_d1;  // sd_init_done 同步 + 下降沿检测
wire             sd_reinit = sd_init_d1 && !sd_init_d0;   // SD 重新初始化开始

// 同步 mem_clk 域的 write_finish_toggle
reg [2:0]        wrfin_tgl_sync;
wire             write_finish_pulse;

wire auto_tick;
wire [1:0] next_from_current;

assign write_en    = bmp_data_wr_en;
assign disp_img_idx = img_idx;   // 图片提交（切显示缓冲区）的同时更新
assign effect_sel  = effect_cnt;
assign fade_ack_pulse = fack_sync[2] ^ fack_sync[1];
assign write_data = {bmp_data[23:16], bmp_data[15:8], bmp_data[7:0], 8'b0};
assign auto_tick  = (auto_cnt == (CLK_FREQ_HZ - 1));
assign next_from_current = next_index_limited(img_idx, img_found_count);
assign write_finish_pulse = wrfin_tgl_sync[2] ^ wrfin_tgl_sync[1];
assign eff_up_pulse = eup_sync[2] ^ eup_sync[1];
assign eff_dn_pulse = edn_sync[2] ^ edn_sync[1];

key_press_debounce #(
    .CLK_FREQ_HZ (CLK_FREQ_HZ),
    .DEBOUNCE_MS (20)
) u_key_next (
    .clk        (clk),
    .rst        (rst),
    .button_in  (key_next),
    .press_pulse(key_next_press)
);

// SW1 开关两级同步（异步输入 → clk 域）
always @(posedge clk or posedge rst) begin
    if (rst) begin
        auto_sw_s0 <= 1'b0;
        auto_sw_s1 <= 1'b0;
    end
    else begin
        auto_sw_s0 <= auto_sw;
        auto_sw_s1 <= auto_sw_s0;
    end
end

function [1:0] next_index_limited;
    input [1:0] cur;
    input [2:0] count;
    begin
        case (count)
            3'd0: next_index_limited = 2'd0;
            3'd1: next_index_limited = 2'd0;
            3'd2: next_index_limited = (cur == 2'd1) ? 2'd0 : (cur + 2'd1);
            3'd3: next_index_limited = (cur == 2'd2) ? 2'd0 : (cur + 2'd1);
            default: next_index_limited = (cur == 2'd3) ? 2'd0 : (cur + 2'd1);
        endcase
    end
endfunction

function [31:0] sector_lut;
    input [1:0] idx;
    begin
        case (idx)
            2'd0: sector_lut = img_sector0;
            2'd1: sector_lut = img_sector1;
            2'd2: sector_lut = img_sector2;
            2'd3: sector_lut = img_sector3;
            default: sector_lut = img_sector0;
        endcase
    end
endfunction

function [1:0] next_buf_lut;
    input [1:0] cur_disp_buf;
    input       valid_now;
    begin
        if (!valid_now)
            next_buf_lut = 2'd0;                 // 首图固定写 buffer0
        else if (cur_disp_buf == 2'd0)
            next_buf_lut = 2'd1;
        else
            next_buf_lut = 2'd0;
    end
endfunction

always @(posedge clk or posedge rst) begin
    if (rst) begin
        wrfin_tgl_sync        <= 3'b000;
        scan_start_pulse      <= 1'b0;
        load_start_pulse      <= 1'b0;
        load_sector           <= 32'd0;
        scan_kicked           <= 1'b0;
        first_image_committed <= 1'b0;
        auto_play_en          <= 1'b0;
        auto_cnt              <= 32'd0;
        img_found_count       <= 3'd0;
        img_idx               <= 2'd0;
        load_idx              <= 2'd0;
        pending_buf_idx       <= 2'd0;
        write_buf_idx         <= 2'd0;
        disp_buf_idx          <= 2'd0;
        img_sector0           <= 32'd0;
        img_sector1           <= 32'd0;
        img_sector2           <= 32'd0;
        img_sector3           <= 32'd0;
        next_req_pending      <= 1'b0;
        load_busy             <= 1'b0;
        load_wr_seen          <= 1'b0;
        load_wd               <= 32'd0;
        load_abort_pulse      <= 1'b0;
        source_done_seen      <= 1'b0;
        write_finish_seen     <= 1'b0;
        dbg_code              <= 3'd0;
        display_valid         <= 1'b0;
        effect_cnt            <= 3'd0;
        wait_fade_ack         <= 1'b0;
        img_commit_tgl        <= 1'b0;
        img_tier              <= 2'd0;
        fack_sync             <= 3'b000;
        eup_sync              <= 3'b000;
        edn_sync              <= 3'b000;
        fade_req              <= 1'b0;
    end else begin
        sd_init_d0 <= sd_init_done;
        sd_init_d1 <= sd_init_d0;
        wrfin_tgl_sync   <= {wrfin_tgl_sync[1:0], write_finish_toggle};
        fack_sync        <= {fack_sync[1:0], fade_ack};
        eup_sync         <= {eup_sync[1:0], eff_up_tgl};
        edn_sync         <= {edn_sync[1:0], eff_dn_tgl};
        scan_start_pulse <= 1'b0;
        load_start_pulse <= 1'b0;

        if (!sd_init_done) begin
            scan_kicked           <= 1'b0;
            first_image_committed <= 1'b0;
            auto_play_en          <= 1'b0;
            auto_cnt              <= 32'd0;
            img_found_count       <= 3'd0;
            img_idx               <= 2'd0;
            load_idx              <= 2'd0;
            pending_buf_idx       <= 2'd0;
            write_buf_idx         <= 2'd0;
            disp_buf_idx          <= 2'd0;
            img_sector0           <= 32'd0;
            img_sector1           <= 32'd0;
            img_sector2           <= 32'd0;
            img_sector3           <= 32'd0;
            next_req_pending      <= 1'b0;
            load_busy             <= 1'b0;
            load_wr_seen          <= 1'b0;
            load_wd               <= 32'd0;
            load_abort_pulse      <= 1'b0;
            source_done_seen      <= 1'b0;
            display_valid         <= 1'b0;
            effect_cnt            <= 3'd0;
            wait_fade_ack         <= 1'b0;
        end else begin
            // 扫描阶段缓存前 4 张图的起始 sector
            if (scan_found_valid) begin
                case (img_found_count)
                    3'd0: img_sector0 <= scan_found_sector;
                    3'd1: img_sector1 <= scan_found_sector;
                    3'd2: img_sector2 <= scan_found_sector;
                    3'd3: img_sector3 <= scan_found_sector;
                    default: ;
                endcase

                if (img_found_count < 3'd4)
                    img_found_count <= img_found_count + 3'd1;
            end

            // 记住 bmp_read 已经把源图送完 FIFO，但还不能切显示，得等整帧写完
            if (load_busy && bmp_ready)
                source_done_seen <= 1'b1;

            // 只有真正收到 write_finish_toggle 脉冲，才进入提交流程：
            // 特效 1/2/3 → 先请求视频域过渡，等 ack 后再提交；
            // 特效 0 → 立即提交（无特效）
            if (load_busy && source_done_seen && write_finish_pulse) begin
                write_finish_seen <= 1'b1;
                if (effect_cnt == 3'd0) begin
                    load_busy             <= 1'b0;
                    source_done_seen      <= 1'b0;
                    load_wr_seen          <= 1'b0;
                    load_wd               <= 32'd0;
                    disp_buf_idx          <= pending_buf_idx;
                    img_idx               <= load_idx;
                    display_valid         <= 1'b1;
                    first_image_committed <= 1'b1;
                    img_commit_tgl        <= ~img_commit_tgl;   // 触发分辨率 OSD
                    img_tier              <= bmp_tier_w;
                end
                else begin
                    fade_req      <= ~fade_req;    // 启动视频域淡出
                    wait_fade_ack <= 1'b1;         // 挂起提交，等淡出完成
                end
            end

            // 淡出完成：提交新图（视频域随后淡入）
            if (wait_fade_ack && fade_ack_pulse) begin
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
                load_wr_seen          <= 1'b0;
                load_wd               <= 32'd0;
                disp_buf_idx          <= pending_buf_idx;
                img_idx               <= load_idx;
                display_valid         <= 1'b1;
                first_image_committed <= 1'b1;
                img_commit_tgl        <= ~img_commit_tgl;       // 触发分辨率 OSD
                img_tier              <= bmp_tier_w;
                wait_fade_ack         <= 1'b0;
            end

            // ---------------- 加载失败恢复（看门狗） ----------------
            // 记录本次加载期间是否发起过写请求
            if (load_busy && write_req)
                load_wr_seen <= 1'b1;

            // 看门狗：加载期间计数（提交后清零）
            // 1024x768 加载约 2 秒 + 仲裁分享余量，10 秒足够；卡住时 10 秒恢复
            if (load_busy && (load_wd < 32'h3B9ACA00))   // 10 秒 @100MHz
                load_wd <= load_wd + 32'd1;
            else if (!load_busy)
                load_wd <= 32'd0;

            // 失败恢复：
            // A) 源端已回空闲（bmp_ready）却从未发起写（头校验失败等）
            // B) 看门狗超时（源端或写通路卡死）
            // C) SD 重新初始化（读失败触发的软复位）：立即中止加载，
            //    并保持重试请求，初始化完成后自动重新加载下一张
            if (load_busy && sd_reinit) begin
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                load_abort_pulse <= 1'b1;
                wait_fade_ack    <= 1'b0;
                next_req_pending <= 1'b1;
            end
            else if (load_busy && (( (load_wd > 32'd10000) && bmp_ready && !load_wr_seen)
                           || (load_wd >= 32'h3B9ACA00))) begin
                load_busy        <= 1'b0;
                source_done_seen <= 1'b0;
                load_abort_pulse <= 1'b1;
                wait_fade_ack    <= 1'b0;
                next_req_pending <= 1'b1;
            end
            else begin
                load_abort_pulse <= 1'b0;
            end



            // ---------------- 诊断码（数码管显示，定位提交流程卡点） ----------------
            // 0 空闲  1 bmp_read 加载中  3 源读完等帧写完成  4 源空闲但未标记(异常)
            // 5 等特效 fade 应答  6 帧已写完但未提交(异常)
            if (!load_busy)
                dbg_code <= 3'd0;
            else if (wait_fade_ack)
                dbg_code <= 3'd5;
            else if (source_done_seen && write_finish_seen)
                dbg_code <= 3'd6;
            else if (source_done_seen)
                dbg_code <= 3'd3;
            else if (bmp_ready)
                dbg_code <= 3'd4;
            else
                dbg_code <= 3'd1;

            // 上电后自动发起一次“扫描前 4 张 BMP”
            if (!scan_kicked && bmp_ready) begin
                scan_start_pulse      <= 1'b1;
                scan_kicked           <= 1'b1;
                first_image_committed <= 1'b0;
                auto_play_en          <= 1'b0;
                auto_cnt              <= 32'd0;
                img_found_count       <= 3'd0;
                img_idx               <= 2'd0;
                load_idx              <= 2'd0;
                pending_buf_idx       <= 2'd0;
                write_buf_idx         <= 2'd0;
                disp_buf_idx          <= 2'd0;
                next_req_pending      <= 1'b0;
                display_valid         <= 1'b0;
                load_busy             <= 1'b0;
                source_done_seen      <= 1'b0;
            end else begin
                // 忙的时候也只记 1 次“下一张”请求，不会累积成连跳两张
                if (key_next_press && scan_done && (img_found_count > 3'd0))
                    next_req_pending <= 1'b1;

                // 转场特效 ±1（OSD 菜单第 5 行 K4 事件，0..5 循环回绕，与 OSD 镜像同源）
                if (eff_up_pulse)
                    effect_cnt <= (effect_cnt == 3'd5) ? 3'd0 : (effect_cnt + 3'd1);
                else if (eff_dn_pulse)
                    effect_cnt <= (effect_cnt == 3'd0) ? 3'd5 : (effect_cnt - 3'd1);

                // SW1 电平直接控制自动轮播：高=自动，低=手动（key1 切图）
                auto_play_en <= auto_sw_s1;

                // 自动播放 1s 计数：只有当前没有写图任务时才计时
                if (scan_done && auto_play_en && display_valid && !load_busy && first_image_committed && (img_found_count > 3'd1)) begin
                    if (auto_tick)
                        auto_cnt <= 32'd0;
                    else
                        auto_cnt <= auto_cnt + 32'd1;
                end else begin
                    auto_cnt <= 32'd0;
                end

                // 首图自动加载到 buffer0
                if (scan_done && !first_image_committed && bmp_ready && !load_busy && (img_found_count != 3'd0)) begin
                    load_idx         <= 2'd0;
                    load_sector      <= img_sector0;
                    pending_buf_idx  <= 2'd0;
                    write_buf_idx    <= 2'd0;
                    load_start_pulse <= 1'b1;
                    write_finish_seen  <= 1'b0;
                    load_busy        <= 1'b1;
                    load_wr_seen     <= 1'b0;
                    load_wd          <= 32'd0;
                    source_done_seen <= 1'b0;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 手动下一张优先：写到“非当前显示”的另一块 buffer
                else if (scan_done && bmp_ready && display_valid && !load_busy && next_req_pending && (img_found_count != 3'd0)) begin
                    load_idx         <= next_from_current;
                    load_sector      <= sector_lut(next_from_current);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    write_finish_seen  <= 1'b0;
                    load_busy        <= 1'b1;
                    load_wr_seen     <= 1'b0;
                    load_wd          <= 32'd0;
                    source_done_seen <= 1'b0;
                    next_req_pending <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
                // 自动播放下一张：同样写到“非当前显示”的另一块 buffer
                else if (scan_done && bmp_ready && display_valid && !load_busy && auto_play_en && auto_tick && (img_found_count > 3'd1)) begin
                    load_idx         <= next_from_current;
                    load_sector      <= sector_lut(next_from_current);
                    pending_buf_idx  <= next_buf_lut(disp_buf_idx, display_valid);
                    write_buf_idx    <= next_buf_lut(disp_buf_idx, display_valid);
                    load_start_pulse <= 1'b1;
                    write_finish_seen  <= 1'b0;
                    load_busy        <= 1'b1;
                    load_wr_seen     <= 1'b0;
                    load_wd          <= 32'd0;
                    source_done_seen <= 1'b0;
                    auto_cnt         <= 32'd0;
                end
            end
        end
    end
end

bmp_read bmp_read_m0(
    .clk                    (clk),
    .rst                    (rst),
    .ready                  (bmp_ready),

    .scan_start             (scan_start_pulse),
    .scan_start_sector      (SCAN_START_SECTOR),
    .scan_max_sector        (SCAN_MAX_SECTOR),
    .scan_target_count      (SCAN_TARGET_COUNT),
    .scan_done              (scan_done),
    .scan_found_valid       (scan_found_valid),
    .scan_found_sector      (scan_found_sector),
    .scan_found_total       (scan_found_total),

    .load_start             (load_start_pulse),
    .load_sector            (load_sector),

    .sd_init_done           (sd_init_done),
    .load_abort             (load_abort_pulse),
    .state_code             (state_code),
    .bmp_width              (bmp_width),
    .bmp_height             (bmp_height),
    .write_req              (write_req),
    .write_req_ack          (write_req_ack),
    .sd_sec_read            (sd_sec_read),
    .sd_sec_read_addr       (sd_sec_read_addr),
    .sd_sec_read_data       (sd_sec_read_data),
    .sd_sec_read_data_valid (sd_sec_read_data_valid),
    .sd_sec_read_end        (sd_sec_read_end),
    .bmp_data_wr_en         (bmp_data_wr_en),
    .bmp_data               (bmp_data),
    .img_tier               (bmp_tier_w)
);

endmodule

module key_press_debounce #(
    parameter integer CLK_FREQ_HZ = 100_000_000,
    parameter integer DEBOUNCE_MS = 20
)(
    input  wire clk,
    input  wire rst,
    input  wire button_in,    // 默认：松开=1，按下=0
    output reg  press_pulse
);

localparam integer DEBOUNCE_CYCLES = (CLK_FREQ_HZ / 1000) * DEBOUNCE_MS;

reg button_sync0;
reg button_sync1;
reg button_stable;
reg [31:0] cnt;

always @(posedge clk or posedge rst) begin
    if (rst) begin
        button_sync0  <= 1'b1;
        button_sync1  <= 1'b1;
        button_stable <= 1'b1;
        cnt           <= 32'd0;
        press_pulse   <= 1'b0;
    end else begin
        button_sync0 <= button_in;
        button_sync1 <= button_sync0;
        press_pulse  <= 1'b0;

        if (button_sync1 == button_stable) begin
            cnt <= 32'd0;
        end else begin
            if (cnt >= DEBOUNCE_CYCLES - 1) begin
                if (button_stable && !button_sync1)
                    press_pulse <= 1'b1;
                button_stable <= button_sync1;
                cnt           <= 32'd0;
            end else begin
                cnt <= cnt + 32'd1;
            end
        end
    end
end

endmodule
