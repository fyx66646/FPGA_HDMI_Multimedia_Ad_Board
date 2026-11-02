// ============================================================================
// bmp_read : TF 卡 BMP 读取（三挡分辨率 + 旧版直通骨架 + 抽取门控）
// 结构回归实测正常的旧版（字节流直通、无 FIFO、无扇区按需），仅增加：
//   1. 三挡头校验（640x480 / 1024x768 / 1280x960，24bit 无压缩）
//   2. 水平/垂直抽取门控（scale 查表 256/160/128，输出恒 640x480 满屏）
//   3. 扫描空转提前结束（图片不足 4 张时约 15 秒出图，不再扫满整卡）
// 数据阶段逐字节流式处理，每 3 字节拼 1 像素；像素完成拍做水平判定
// （acc_h），行末做垂直判定（acc_v），wr_en 与像素数据同拍（与旧版一致）。
// ============================================================================
module bmp_read(
    input                       clk,
    input                       rst,
    output                      ready,

    input                       scan_start,
    input  [31:0]               scan_start_sector,
    input  [31:0]               scan_max_sector,
    input  [2:0]                scan_target_count,
    output reg                  scan_done,
    output reg                  scan_found_valid,
    output reg [31:0]           scan_found_sector,
    output reg [2:0]            scan_found_total,

    input                       load_start,
    input  [31:0]               load_sector,

    input                       sd_init_done,
    input                       load_abort,
    output reg [3:0]            state_code,
    input  [15:0]               bmp_width,
    input  [15:0]               bmp_height,

    output reg                  write_req,
    input                       write_req_ack,

    output reg                  sd_sec_read,
    output reg [31:0]           sd_sec_read_addr,
    input  [7:0]                sd_sec_read_data,
    input                       sd_sec_read_data_valid,
    input                       sd_sec_read_end,

    output reg                  bmp_data_wr_en,
    output reg [23:0]           bmp_data,

    // 源图分辨率挡（头校验通过时随 width 定档，供切图后 OSD 显示分辨率）
    output wire [1:0]           img_tier         // 0=640x480 1=1024x768 2=1280x960
);

localparam ST_IDLE      = 3'd0;
localparam ST_SCAN      = 3'd1;
localparam ST_LOAD_HDR  = 3'd2;
localparam ST_LOAD_WAIT = 3'd3;
localparam ST_LOAD_DATA = 3'd4;

reg [2:0]  state;
reg [9:0]  rd_cnt;

reg [7:0]  header_0;
reg [7:0]  header_1;
reg [31:0] file_len;
reg [31:0] tf_data_sectors;
reg [31:0] pixel_offset;
reg [31:0] width;
reg [31:0] height;
reg [15:0] bit_count;
reg [31:0] compression;

reg [31:0] scan_sector;
reg [31:0] load_sector_latched;
reg [31:0] bmp_len_cnt;
reg [1:0]  bmp_byte_idx;
reg [31:0] idle_cnt;       // 扫描空转计数

// ---- 三挡缩放参数 ----
function [9:0] scale_lut;
    input [15:0] w;
    begin
        case (w)
            16'd640 : scale_lut = 10'd256;
            16'd1024: scale_lut = 10'd160;
            default : scale_lut = 10'd128;      // 1280
        endcase
    end
endfunction

reg [9:0]  scale;
reg        load_go;        // 状态机发出的加载启动脉冲（数据块初始化用）
reg [15:0] src_col;        // 当前源行已收像素数
reg [8:0]  acc_h;          // 水平抽取累加器
reg [8:0]  acc_v;          // 垂直抽取累加器
reg        row_active;     // 当前源行是否输出

wire header_match;
wire bmp_data_valid;
wire [31:0] file_sector_count;
wire [31:0] next_scan_sector_if_match;
wire [31:0] next_scan_sector_if_miss;

assign ready = (state == ST_IDLE);
// 三挡校验
assign header_match = (header_0 == "B") &&
                      (header_1 == "M") &&
                      (bit_count    == 16'd24) &&
                      (compression  == 32'd0) &&
                      (height[31] == 1'b0) &&
                      (((width == 32'd640)  && (height == 32'd480)) ||
                       ((width == 32'd1024) && (height == 32'd768)) ||
                       ((width == 32'd1280) && (height == 32'd960)));
assign bmp_data_valid = (sd_sec_read_data_valid == 1'b1) &&
                        (bmp_len_cnt >= pixel_offset) &&
                        (bmp_len_cnt <  file_len);
assign file_sector_count = (file_len == 32'd0) ? 32'd1 : ((file_len + 32'd511) >> 9);
// 分辨率挡：仅校验通过的三挡（width 高 16 位为 0 时三挡均不匹配，输出 0 无碍）
assign img_tier = (width == 32'd640)  ? 2'd0 :
                  (width == 32'd1024) ? 2'd1 : 2'd2;
// ---- 扫描步进流水预计算（2026-10-25 时序修复，与 sd_wav_reader 同款）----
// 原组合链 file_len→跨度加法→scan_sector+跨度→比较→多路选择→scan_sector
// 曾是 SD 域主要 setup 违例。拆 5 级寄存，在扇区头部 rd_cnt 35..40 期间
// 逐级完成（全部字段 rd_cnt33 后稳定），read_end 只剩寄存器直通与 +1 递增。
// 语义说明：TF 音频头越界时原逻辑退化为 miss(+1)，新逻辑同（tf_sel&&exceed
// 落入 miss 分支）；BMP 命中时仍按原样跳转（跨上限则同时收工，行为不变）。
wire [31:0] tf_skip = 32'd1 + tf_data_sectors;
reg [31:0] bmp_cnt_r;         // rd_cnt35 寄存：BMP 文件扇区跨度
reg [31:0] tf_skip_r;         // rd_cnt36 寄存：音频轨跨度 1+tf_data_sectors
reg [31:0] sel_skip_r;        // rd_cnt38 寄存：选中跨度
reg         is_match_r;       // rd_cnt38 寄存：BMP 命中
reg         tf_sel_r;         // rd_cnt38 寄存：TF 音频头命中（未含越界检查）
reg [31:0] scan_pre_sum;      // rd_cnt39 寄存：scan_sector + sel_skip_r
reg [31:0] scan_pre_next;     // rd_cnt40 寄存：最终下一扇区（越界→max+1）
reg         scan_pre_exceed;  // rd_cnt40 寄存：越界/回绕

// ---------------- 头解析 ----------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        rd_cnt <= 10'd0;
    end else if ((state == ST_SCAN) || (state == ST_LOAD_HDR)) begin
        if (sd_sec_read_data_valid)
            rd_cnt <= rd_cnt + 10'd1;
        else if (sd_sec_read_end)
            rd_cnt <= 10'd0;
    end else begin
        rd_cnt <= 10'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        header_0     <= 8'd0;
        header_1     <= 8'd0;
        file_len     <= 32'd0;
        tf_data_sectors <= 32'd0;
        pixel_offset <= 32'd54;
        width        <= 32'd0;
        height       <= 32'd0;
        bit_count    <= 16'd0;
        compression  <= 32'd0;
    end else if (((state == ST_SCAN) || (state == ST_LOAD_HDR)) && sd_sec_read_data_valid) begin
        case (rd_cnt)
            10'd0 : header_0 <= sd_sec_read_data;
            10'd1 : header_1 <= sd_sec_read_data;
            10'd2 : file_len[7:0] <= sd_sec_read_data;
            10'd3 : file_len[15:8] <= sd_sec_read_data;
            10'd4 : file_len[23:16] <= sd_sec_read_data;
            10'd5 : file_len[31:24] <= sd_sec_read_data;
            10'd8 : tf_data_sectors[7:0]  <= sd_sec_read_data;
            10'd9 : tf_data_sectors[15:8] <= sd_sec_read_data;
            10'd10: begin
                pixel_offset[7:0]        <= sd_sec_read_data;
                tf_data_sectors[23:16]   <= sd_sec_read_data;
            end
            10'd11: begin
                pixel_offset[15:8]       <= sd_sec_read_data;
                tf_data_sectors[31:24]   <= sd_sec_read_data;
            end
            10'd12: pixel_offset[23:16] <= sd_sec_read_data;
            10'd13: pixel_offset[31:24] <= sd_sec_read_data;
            10'd18: width[7:0] <= sd_sec_read_data;
            10'd19: width[15:8] <= sd_sec_read_data;
            10'd20: width[23:16] <= sd_sec_read_data;
            10'd21: width[31:24] <= sd_sec_read_data;
            10'd22: height[7:0] <= sd_sec_read_data;
            10'd23: height[15:8] <= sd_sec_read_data;
            10'd24: height[23:16] <= sd_sec_read_data;
            10'd25: height[31:24] <= sd_sec_read_data;
            10'd28: bit_count[7:0] <= sd_sec_read_data;
            10'd29: bit_count[15:8] <= sd_sec_read_data;
            10'd30: compression[7:0] <= sd_sec_read_data;
            10'd31: compression[15:8] <= sd_sec_read_data;
            10'd32: compression[23:16] <= sd_sec_read_data;
            10'd33: compression[31:24] <= sd_sec_read_data;
            default: ;
        endcase

        // 扫描步进流水预计算：各级在字段稳定后的对应字节拍寄存
        // （ST_LOAD_HDR 期间同样无害地重复装载；选择级与加法级分拍）
        if (rd_cnt == 10'd35)
            bmp_cnt_r <= file_sector_count;
        if (rd_cnt == 10'd36)
            tf_skip_r <= tf_skip;
        if (rd_cnt == 10'd38) begin
            is_match_r <= header_match;
            tf_sel_r   <= (header_0 == "T") && (header_1 == "F") &&
                          (tf_data_sectors != 32'd0);
            sel_skip_r <= header_match ? bmp_cnt_r :
                          ((header_0 == "T") && (header_1 == "F") &&
                           (tf_data_sectors != 32'd0)) ? tf_skip_r : 32'd1;
        end
        if (rd_cnt == 10'd39)
            scan_pre_sum <= scan_sector + sel_skip_r;
        if (rd_cnt == 10'd40) begin
            scan_pre_exceed <= !((scan_pre_sum > scan_sector) &&
                                 (scan_pre_sum <= scan_max_sector));
            scan_pre_next   <= ((scan_pre_sum > scan_sector) &&
                                (scan_pre_sum <= scan_max_sector))
                                ? scan_pre_sum : (scan_max_sector + 32'd1);
        end
    end
end

// ---------------- 字节流计数与像素拼接（旧版骨架） ----------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_len_cnt <= 32'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (sd_sec_read_data_valid)
            bmp_len_cnt <= bmp_len_cnt + 32'd1;
    end else begin
        bmp_len_cnt <= 32'd0;
    end
end

always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_byte_idx <= 2'd0;
    end else if (state == ST_LOAD_DATA) begin
        if (bmp_data_valid)
            bmp_byte_idx <= (bmp_byte_idx == 2'd2) ? 2'd0 : (bmp_byte_idx + 2'd1);
    end else begin
        bmp_byte_idx <= 2'd0;
    end
end

// 水平抽取判定（组合，读老 acc_h）
wire [9:0] acc_h_next = {1'b0, acc_h} + {1'b0, scale};
wire       acc_h_hit  = (acc_h_next >= 10'd256);

// ---------------- 数据阶段输出（旧版骨架 + 抽取门控） ----------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        bmp_data_wr_en <= 1'b0;
        bmp_data       <= 24'd0;
        src_col        <= 16'd0;
        acc_h          <= 9'd0;
        acc_v          <= 9'd0;
        row_active     <= 1'b0;
    end else if (load_go) begin
        // 加载启动：初始化抽取状态（预判首个源行）
        bmp_data_wr_en <= 1'b0;
        src_col   <= 16'd0;
        acc_h     <= 9'd0;
        acc_v     <= (scale >= 10'd256) ? (scale - 10'd256) : scale;
        row_active<= (scale >= 10'd256);
    end else if (state == ST_LOAD_DATA) begin
        bmp_data_wr_en <= 1'b0;
        if (bmp_data_valid) begin
            case (bmp_byte_idx)
                2'd0: begin
                    bmp_data[7:0]  <= sd_sec_read_data;      // B
                end
                2'd1: begin
                    bmp_data[15:8] <= sd_sec_read_data;      // G
                end
                default: begin
                    // 像素完成拍：R 字节 + 水平抽取判定 + 行计数
                    bmp_data[23:16] <= sd_sec_read_data;     // R
                    acc_h <= acc_h_hit ? (acc_h_next - 10'd256) : acc_h_next;
                    bmp_data_wr_en <= row_active && acc_h_hit;
                    if (src_col == width[15:0] - 1) begin
                        // 行末：垂直抽取判定（下一个源行）
                        src_col     <= 16'd0;
                        acc_h       <= 9'd0;
                        if (acc_v + {1'b0, scale} >= 10'd256) begin
                            acc_v      <= acc_v + {1'b0, scale} - 10'd256;
                            row_active <= 1'b1;
                        end
                        else begin
                            acc_v      <= acc_v + {1'b0, scale};
                            row_active <= 1'b0;
                        end
                    end
                    else begin
                        src_col <= src_col + 16'd1;
                    end
                end
            endcase
        end
    end else begin
        bmp_data_wr_en <= 1'b0;
    end
end

// ---------------- 主状态机 ----------------
always @(posedge clk or posedge rst) begin
    if (rst) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        idle_cnt          <= 32'd0;
        load_sector_latched <= 32'd0;
        load_go           <= 1'b0;
    end else if (!sd_init_done) begin
        state             <= ST_IDLE;
        state_code        <= 4'd0;
        sd_sec_read       <= 1'b0;
        sd_sec_read_addr  <= 32'd0;
        write_req         <= 1'b0;
        scan_done         <= 1'b0;
        scan_found_valid  <= 1'b0;
        scan_found_sector <= 32'd0;
        scan_found_total  <= 3'd0;
        scan_sector       <= 32'd0;
        idle_cnt          <= 32'd0;
        load_sector_latched <= 32'd0;
    end else if (load_abort) begin
        state        <= ST_IDLE;
        state_code   <= 4'd1;
        sd_sec_read  <= 1'b0;
        write_req    <= 1'b0;
    end else begin
        scan_found_valid <= 1'b0;
        load_go          <= 1'b0;

        case (state)
            ST_IDLE: begin
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
                if (scan_start) begin
                    scan_done        <= 1'b0;
                    scan_found_total <= 3'd0;
                    scan_sector      <= scan_start_sector;
                    sd_sec_read_addr <= scan_start_sector;
                    idle_cnt         <= 32'd0;
                    state            <= ST_SCAN;
                end else if (load_start) begin
                    load_sector_latched <= load_sector;
                    sd_sec_read_addr    <= load_sector;
                    state               <= ST_LOAD_HDR;
                end
            end

            ST_SCAN: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (is_match_r) begin
                        scan_found_valid  <= 1'b1;
                        scan_found_sector <= scan_sector;
                        scan_found_total  <= scan_found_total + 3'd1;
                        idle_cnt <= 32'd0;
                        if ((scan_found_total + 3'd1 >= scan_target_count) ||
                            (scan_pre_next > scan_max_sector)) begin
                            scan_done        <= 1'b1;
                            state            <= ST_IDLE;
                            sd_sec_read_addr <= scan_pre_next;
                            scan_sector      <= scan_pre_next;
                        end else begin
                            sd_sec_read_addr <= scan_pre_next;
                            scan_sector      <= scan_pre_next;
                        end
                    end else begin
                        if (tf_sel_r && !scan_pre_exceed) begin
                            idle_cnt         <= 32'd0;
                            sd_sec_read_addr <= scan_pre_next;
                            scan_sector      <= scan_pre_next;
                        end else if (scan_sector >= scan_max_sector) begin
                            scan_done <= 1'b1;
                            state     <= ST_IDLE;
                        end
                        // 空转 50000 扇区（约 15 秒）提前结束：图片不足 4 张时不扫满整卡
                        else if (idle_cnt >= 32'd50000) begin
                            scan_done <= 1'b1;
                            state     <= ST_IDLE;
                        end
                        else begin
                            idle_cnt         <= idle_cnt + 32'd1;
                            sd_sec_read_addr <= scan_sector + 32'd1;
                            scan_sector      <= scan_sector + 32'd1;
                        end
                    end
                end
            end

            ST_LOAD_HDR: begin
                state_code  <= 4'd2;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (header_match) begin
                        write_req        <= 1'b1;
                        sd_sec_read_addr <= load_sector_latched;
                        scale <= scale_lut(width[15:0]);
                        state <= ST_LOAD_WAIT;
                    end else begin
                        state <= ST_IDLE;
                    end
                end
            end

            ST_LOAD_WAIT: begin
                state_code <= 4'd3;
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    load_go   <= 1'b1;      // 数据块初始化抽取状态
                    state     <= ST_LOAD_DATA;
                end
            end

            ST_LOAD_DATA: begin
                state_code  <= 4'd4;
                sd_sec_read <= 1'b1;
                if (sd_sec_read_end) begin
                    sd_sec_read <= 1'b0;
                    if (bmp_len_cnt >= file_len) begin
                        state <= ST_IDLE;
                    end else begin
                        sd_sec_read_addr <= sd_sec_read_addr + 32'd1;
                    end
                end
            end

            default: begin
                state       <= ST_IDLE;
                state_code  <= 4'd1;
                sd_sec_read <= 1'b0;
                write_req   <= 1'b0;
            end
        endcase
    end
end

endmodule
