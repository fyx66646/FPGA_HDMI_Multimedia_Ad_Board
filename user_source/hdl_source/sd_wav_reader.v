// ============================================================================
// sd_wav_reader : TF 卡多轨音频读取（v4 中文字幕版）
// 基座 = 实测正常的 v3 备份版本（扫描收集 4 轨 / 循环回跳重装计数器）。
// v4：头扇区（含字幕单元 + 汉字数）之后若有字库扇区（每汉字 32 字节 =
// 16 行 x 2 字节小端），逐扇区流式灌入 font_cn_ram，全部灌完才翻转
// txt_toggle 发布字幕；循环回跳改用记录的 PCM 起始扇区/扇区数
// （pcm_base_r/pcm_len_r），不再每圈重读字库。
// 文件布局：[头扇区][字库扇区 0..N][PCM 扇区]，N = ceil(汉字数/16)。
// ============================================================================
module sd_wav_reader #(
    parameter [31:0] SCAN_START_SECTOR = 32'd0,
    parameter [31:0] SCAN_MAX_SECTOR   = 32'd131071,
    parameter [11:0] WM_HI             = 12'd1408,      // 低于此值预取
    parameter [11:0] WM_LO             = 12'd512        // 低于此值置 urgent 请求仲裁插队
)(
    input  wire        clk,               // sd_card_clk 100MHz
    input  wire        rst,               // 高有效

    input  wire        sd_init_done,
    input  wire [1:0]  I_track_sel,       // 当前显示图片编号（音频跟随其切换）

    // 扇区读主口（接 sd_read_arbiter）
    output reg         sd_sec_read,
    output reg  [31:0] sd_sec_read_addr,
    input  wire [7:0]  sd_sec_read_data,
    input  wire        sd_sec_read_data_valid,
    input  wire        sd_sec_read_end,

    // 异步 FIFO 写侧（同时钟域）
    input  wire [11:0] fifo_usedw,
    input  wire        fifo_full,
    output reg         fifo_we,
    output reg  [15:0] fifo_di,

    // 字幕文字（写入 text_ram_dp，进入播放/切轨时从头扇区抓取）
    output reg         txt_we,
    output reg  [7:0]  txt_waddr,
    output reg  [7:0]  txt_wdata,
    output reg         txt_toggle,       // 文字+字库抓完完成后翻转，供 video 域锁存
    output reg  [7:0]  txt_len_o,        // 单元数（0=无字幕），保持稳定

    // 汉字字库（头扇区之后的字库扇区流式灌入 font_cn_dp RAM）
    output reg         font_we,
    output reg  [10:0] font_waddr,       // 字地址 = 汉字序号*16 + 行
    output reg  [15:0] font_wdata,       // 行点阵，bit15=最左像素

    output wire        O_stream_mode,    // 正在播放音频数据
    output wire        O_urgent,         // FIFO 低于低水位，请求仲裁优先
    output reg  [3:0]  O_state_code,     // 0空闲 1扫描 2播放 3卡上无音频 4当前图片无对应音频
    output wire [2:0]  O_found_count,    // 诊断：已找到的音频轨道数
    output wire [1:0]  O_cur_track       // 诊断：当前播放轨道
);

    localparam ST_IDLE   = 3'd0;
    localparam ST_SCAN   = 3'd1;
    localparam ST_STREAM = 3'd2;
    localparam ST_DONE   = 3'd3;

    // "TFWAVE01"
    localparam [63:0] MAGIC = 64'h5446574156453031;

    reg [2:0]  state;
    reg        sec_busy;        // 一个扇区读在途

    // 扫描
    reg [31:0] scan_sector;
    reg [63:0] magic_shift;
    reg [31:0] hdr_rate;
    reg [15:0] hdr_channels;
    reg [15:0] hdr_bits;
    reg [31:0] data_sectors;    // 头字节8-11：除头外总扇区数（字库+PCM）
    reg [7:0]  hdr_track;       // 头字节20：轨道号
    reg [31:0] skip_len;        // 扫描遇 BMP 时取其文件长度（字节2-5），整段跳过

    // 四轨登记
    reg [31:0] hdr_sec0, hdr_sec1, hdr_sec2, hdr_sec3;
    reg [31:0] dsec0,   dsec1,   dsec2,   dsec3;
    reg [3:0]  found;
    reg [2:0]  found_count;

    // 播放
    reg [1:0]  cur_track;       // 正在播放的轨道
    reg        track_active;    // 当前轨道存在且在播（0=静音待命）
    reg [31:0] data_addr;       // 下一个要读的数据扇区
    reg [31:0] data_left;       // 本轮循环剩余扇区数
    reg        scan_exit_pending;
    reg        fetch_hdr;       // 正在读头扇区抓字幕（数据不进音频 FIFO）
    reg  [7:0] txt_len_hold;    // 头字节21：字幕单元数
    reg  [7:0] glyph_cnt_hold;  // 头字节22：汉字子集字数（≤128）
    reg  [3:0] font_left;       // 待读字库扇区数（0=无，最多 8）
    reg [10:0] font_wa;         // 字库字写地址（每汉字 16 字）
    reg [7:0]  font_lsb;        // 拼字库 16bit 小端用
    reg [31:0] pcm_base_r;      // 本轨 PCM 起始扇区（头+字库之后）
    reg [31:0] pcm_len_r;       // 本轨 PCM 扇区数

    // 字节流
    reg [8:0]  byte_idx;        // 扇区内字节位置 0..511
    reg [7:0]  lsb_hold;        // 拼 16bit 小端用

    // ---------------- 头部解析组合逻辑 ----------------
    wire magic_hit = (magic_shift == MAGIC);
    wire hdr_valid = magic_hit && (hdr_rate == 32'd48000) &&
                     (hdr_channels == 16'd2) && (hdr_bits == 16'd16) &&
                     (data_sectors != 32'd0) && (data_sectors < 32'h1000_0000) &&
                     (hdr_track <= 32'd3);
    wire [1:0]  hdr_idx    = hdr_track[1:0];
    wire        new_find   = hdr_valid && !found[hdr_idx];
    wire [2:0]  found_cnt_w= found_count + (new_find ? 3'd1 : 3'd0);

    wire is_bmp        = (magic_shift[63:56] == 8'h42) && (magic_shift[55:48] == 8'h4D);
    // ---- 扫描步进流水预计算（2026-10-25 时序修复，已知问题#4）----
    // 原组合链 skip_len→32位加→越界比较→三路选择→scan_sector 共 11 级逻辑
    // （慢角曾 -2.2ns）。拆 4 级寄存，在扇区头部字节期间逐级完成（字段在
    // byte21 后全部稳定），read_end 只剩寄存器直通；扇区 512 字节，余量巨大。
    // 语义说明：BMP 跨越扫描上限时原逻辑退化为 +1 逐扇区爬过文件尾部，
    // 新逻辑直接收工——上限之外本无内容可扫，最终结果等价。
    reg [31:0] bmp_skip_r;        // byte6 寄存：BMP 文件扇区跨度
    reg [31:0] aud_skip_r;        // byte12 寄存：音频轨跨度 1+data_sectors
    reg [31:0] skip_sel_r;        // byte22 寄存：选中跨度（选择级与加法级分拍）
    reg [31:0] scan_pre_sum;      // byte23 寄存：scan_sector + skip_sel_r
    reg [31:0] scan_pre_next;     // byte24 寄存：最终下一扇区（越界→MAX+1）
    reg         scan_pre_exceed;  // byte24 寄存：越界/回绕标志
    reg         scan_exit4_r;     // byte24 寄存：本扇区后找满 4 轨（read_end 退出条件预寄存）

    // 音频 FIFO full 标志打一拍：切断 IP 内部 写指针→full 组合链直接
    // 驱动数据通路门控的时序路径（水位预取下 FIFO 远离满，晚一拍无害）
    reg fifo_full_s1;
    always @(posedge clk or posedge rst) begin
        if (rst)
            fifo_full_s1 <= 1'b0;
        else
            fifo_full_s1 <= fifo_full;
    end

    function [31:0] hdr_sec_lut;
        input [1:0] idx;
        begin
            case (idx)
                2'd0: hdr_sec_lut = hdr_sec0;
                2'd1: hdr_sec_lut = hdr_sec1;
                2'd2: hdr_sec_lut = hdr_sec2;
                default: hdr_sec_lut = hdr_sec3;
            endcase
        end
    endfunction

    function [31:0] dsec_lut;
        input [1:0] idx;
        begin
            case (idx)
                2'd0: dsec_lut = dsec0;
                2'd1: dsec_lut = dsec1;
                2'd2: dsec_lut = dsec2;
                default: dsec_lut = dsec3;
            endcase
        end
    endfunction

    assign O_stream_mode = (state == ST_STREAM) && track_active;
    assign O_urgent      = (state == ST_STREAM) && track_active && (fifo_usedw <= WM_LO);
    assign O_found_count = found_count;
    assign O_cur_track   = cur_track;

    wire        sel_found = found[I_track_sel];
    wire [31:0] sel_hdr   = hdr_sec_lut(I_track_sel);
    wire [31:0] sel_dsec  = dsec_lut(I_track_sel);

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state         <= ST_IDLE;
            sec_busy      <= 1'b0;
            scan_sector   <= SCAN_START_SECTOR;
            magic_shift   <= 64'd0;
            hdr_rate      <= 32'd0;
            hdr_channels  <= 16'd0;
            hdr_bits      <= 16'd0;
            data_sectors  <= 32'd0;
            hdr_track     <= 8'd0;
            skip_len      <= 32'd0;
            hdr_sec0      <= 32'd0;  hdr_sec1 <= 32'd0;
            hdr_sec2      <= 32'd0;  hdr_sec3 <= 32'd0;
            dsec0         <= 32'd0;  dsec1    <= 32'd0;
            dsec2         <= 32'd0;  dsec3    <= 32'd0;
            found         <= 4'd0;
            found_count   <= 3'd0;
            cur_track     <= 2'd0;
            track_active  <= 1'b0;
            data_addr     <= 32'd0;
            data_left     <= 32'd0;
            scan_exit_pending <= 1'b0;
            fetch_hdr     <= 1'b0;
            txt_len_hold  <= 8'd0;
            glyph_cnt_hold<= 8'd0;
            font_left     <= 4'd0;
            font_wa       <= 11'd0;
            font_lsb      <= 8'd0;
            pcm_base_r    <= 32'd0;
            pcm_len_r     <= 32'd0;
            byte_idx      <= 9'd0;
            lsb_hold      <= 8'd0;
            sd_sec_read   <= 1'b0;
            sd_sec_read_addr <= SCAN_START_SECTOR;
            fifo_we       <= 1'b0;
            fifo_di       <= 16'd0;
            txt_we        <= 1'b0;
            txt_waddr     <= 8'd0;
            txt_wdata     <= 8'd0;
            font_we       <= 1'b0;
            font_waddr    <= 11'd0;
            font_wdata    <= 16'd0;
            txt_toggle    <= 1'b0;
            txt_len_o     <= 8'd0;
            O_state_code  <= 4'd0;
        end
        else begin
            fifo_we <= 1'b0;
            txt_we  <= 1'b0;
            font_we <= 1'b0;

            if (!sd_init_done) begin
                state            <= ST_IDLE;
                sec_busy         <= 1'b0;
                scan_sector      <= SCAN_START_SECTOR;
                magic_shift      <= 64'd0;
                hdr_rate         <= 32'd0;
                hdr_channels     <= 16'd0;
                hdr_bits         <= 16'd0;
                data_sectors     <= 32'd0;
                hdr_track        <= 8'd0;
                skip_len         <= 32'd0;
                hdr_sec0         <= 32'd0;  hdr_sec1 <= 32'd0;
                hdr_sec2         <= 32'd0;  hdr_sec3 <= 32'd0;
                dsec0            <= 32'd0;  dsec1    <= 32'd0;
                dsec2            <= 32'd0;  dsec3    <= 32'd0;
                found            <= 4'd0;
                found_count      <= 3'd0;
                cur_track        <= 2'd0;
                track_active     <= 1'b0;
                scan_exit_pending<= 1'b0;
                fetch_hdr        <= 1'b0;
                font_left        <= 4'd0;
                byte_idx         <= 9'd0;
                sd_sec_read      <= 1'b0;
                sd_sec_read_addr <= SCAN_START_SECTOR;
                O_state_code     <= 4'd0;
            end
            else begin
                case (state)
                    ST_IDLE: begin
                        O_state_code  <= 4'd0;
                        scan_sector   <= SCAN_START_SECTOR;
                        magic_shift   <= 64'd0;
                        state         <= ST_SCAN;
                    end

                    ST_SCAN: begin
                        O_state_code <= 4'd1;

                        // 扫描结束：延迟一拍进入播放（等 found/hdr_sec 登记落定）
                        if (scan_exit_pending) begin
                            cur_track    <= I_track_sel;
                            track_active <= sel_found;
                            if (sel_found) begin
                                data_addr <= sel_hdr;      // 先读头扇区抓字幕+字库
                                data_left <= sel_dsec;
                                fetch_hdr  <= 1'b1;
                            end
                            font_left <= 4'd0;
                            state <= ST_STREAM;
                        end

                        if (!sec_busy && !scan_exit_pending) begin
                            sd_sec_read      <= 1'b1;
                            sd_sec_read_addr <= scan_sector;
                            sec_busy         <= 1'b1;
                        end

                        if (sd_sec_read_data_valid) begin
                            if (byte_idx < 9'd8)
                                magic_shift <= {magic_shift[55:0], sd_sec_read_data};
                            case (byte_idx)
                                9'd2 : skip_len[7:0]       <= sd_sec_read_data;
                                9'd3 : skip_len[15:8]      <= sd_sec_read_data;
                                9'd4 : skip_len[23:16]     <= sd_sec_read_data;
                                9'd5 : skip_len[31:24]     <= sd_sec_read_data;
                                9'd8 : data_sectors[7:0]   <= sd_sec_read_data;
                                9'd9 : data_sectors[15:8]  <= sd_sec_read_data;
                                9'd10: data_sectors[23:16] <= sd_sec_read_data;
                                9'd11: data_sectors[31:24] <= sd_sec_read_data;
                                9'd12: hdr_rate[7:0]       <= sd_sec_read_data;
                                9'd13: hdr_rate[15:8]      <= sd_sec_read_data;
                                9'd14: hdr_rate[23:16]     <= sd_sec_read_data;
                                9'd15: hdr_rate[31:24]     <= sd_sec_read_data;
                                9'd16: hdr_channels[7:0]   <= sd_sec_read_data;
                                9'd17: hdr_channels[15:8]  <= sd_sec_read_data;
                                9'd18: hdr_bits[7:0]       <= sd_sec_read_data;
                                9'd19: hdr_bits[15:8]      <= sd_sec_read_data;
                                9'd20: hdr_track           <= sd_sec_read_data;
                                default: ;
                            endcase

                            // 流水预计算：各级在字段稳定后的对应字节拍寄存。
                            // 选择级(byte22)与加法级(byte23)分拍——is_bmp 比较+mux
                            // 与 32 位加法同拍曾 -0.14ns,拆开后各自 <6 级逻辑
                            if (byte_idx == 9'd6)
                                bmp_skip_r <= (skip_len <= 32'd512) ? 32'd1
                                                : ((skip_len + 32'd511) >> 9);
                            if (byte_idx == 9'd12)
                                aud_skip_r <= 32'd1 + data_sectors;
                            if (byte_idx == 9'd22)
                                skip_sel_r <= hdr_valid ? aud_skip_r :
                                              is_bmp     ? bmp_skip_r : 32'd1;
                            if (byte_idx == 9'd23)
                                scan_pre_sum <= scan_sector + skip_sel_r;
                            if (byte_idx == 9'd24) begin
                                scan_pre_exceed <= !((scan_pre_sum > scan_sector) &&
                                                     (scan_pre_sum <= SCAN_MAX_SECTOR));
                                scan_pre_next   <= ((scan_pre_sum > scan_sector) &&
                                                    (scan_pre_sum <= SCAN_MAX_SECTOR))
                                                    ? scan_pre_sum : (SCAN_MAX_SECTOR + 32'd1);
                                scan_exit4_r    <= (found_cnt_w == 3'd4);
                            end

                            byte_idx <= byte_idx + 9'd1;
                        end

                        if (sd_sec_read_end) begin
                            sd_sec_read <= 1'b0;       // 撤销一拍，给仲裁器切换窗口
                            sec_busy    <= 1'b0;
                            byte_idx    <= 9'd0;

                            if (new_find) begin
                                case (hdr_idx)
                                    2'd0: begin hdr_sec0 <= scan_sector; dsec0 <= data_sectors; end
                                    2'd1: begin hdr_sec1 <= scan_sector; dsec1 <= data_sectors; end
                                    2'd2: begin hdr_sec2 <= scan_sector; dsec2 <= data_sectors; end
                                    default: begin hdr_sec3 <= scan_sector; dsec3 <= data_sectors; end
                                endcase
                                found[hdr_idx] <= 1'b1;
                                found_count    <= found_cnt_w;
                            end

                            if (scan_exit4_r || scan_pre_exceed) begin
                                scan_exit_pending <= 1'b1;
                            end
                            else begin
                                scan_sector <= scan_pre_next;
                            end
                            magic_shift <= 64'd0;
                        end
                    end

                    ST_STREAM: begin
                        O_state_code <= track_active ? 4'd2 : 4'd4;

                        // 图片切换跟随：空闲时立即切轨（含从静音中恢复）
                        if ((I_track_sel != cur_track) && !sec_busy) begin
                            cur_track    <= I_track_sel;
                            track_active <= sel_found;
                            if (sel_found) begin
                                data_addr <= sel_hdr;      // 先读头扇区抓字幕+字库
                                data_left <= sel_dsec;
                                fetch_hdr  <= 1'b1;
                            end
                            font_left <= 4'd0;
                        end

                        // FIFO 降到高水位以下才要新扇区（仅活动轨道）
                        if (track_active && !sec_busy && (fifo_usedw <= WM_HI)) begin
                            sd_sec_read      <= 1'b1;
                            sd_sec_read_addr <= data_addr;
                            sec_busy         <= 1'b1;
                        end

                        // 头扇区抓字幕+汉字数；字库扇区拼 16bit 小端灌 font_cn_ram；
                        // 其余（PCM）扇区两字节拼 16bit 进音频 FIFO
                        if (sd_sec_read_data_valid) begin
                            if (fetch_hdr) begin
                                if (byte_idx == 9'd21)
                                    txt_len_hold <= (sd_sec_read_data > 8'd100) ?
                                                    8'd100 : sd_sec_read_data;
                                else if (byte_idx == 9'd22)
                                    glyph_cnt_hold <= (sd_sec_read_data > 8'd128) ?
                                                      8'd128 : sd_sec_read_data;
                                else if ((byte_idx >= 9'd32) && (byte_idx <= 9'd131)) begin
                                    txt_we    <= 1'b1;
                                    txt_waddr <= byte_idx[7:0] - 8'd32;
                                    txt_wdata <= sd_sec_read_data;
                                end
                            end
                            else if (font_left != 4'd0) begin
                                // 字库 blob 连续跨扇区、总长恒为偶数，按扇区内
                                // 字节奇偶配对即可保持对齐
                                if (byte_idx[0] == 1'b0)
                                    font_lsb <= sd_sec_read_data;
                                else begin
                                    font_we    <= 1'b1;
                                    font_waddr <= font_wa;
                                    font_wdata <= {sd_sec_read_data, font_lsb};
                                    font_wa    <= font_wa + 11'd1;
                                end
                            end
                            else begin
                                if (byte_idx[0] == 1'b0)
                                    lsb_hold <= sd_sec_read_data;
                                else if (!fifo_full_s1) begin
                                    // full 打一拍再用（2026-10-25 时序修复）：
                                    // 原组合路径 音频FIFO写指针→full→本门控→fifo_di.ce
                                    // 曾 -0.41ns。水位预取下 FIFO 远离满，晚一拍无影响
                                    fifo_we <= 1'b1;
                                    fifo_di <= {sd_sec_read_data, lsb_hold};
                                end
                            end
                            byte_idx <= byte_idx + 9'd1;
                        end

                        if (sd_sec_read_end) begin
                            sd_sec_read <= 1'b0;
                            sec_busy    <= 1'b0;
                            byte_idx    <= 9'd0;

                            if (I_track_sel != cur_track) begin
                                // 扇区边界切轨：不推进旧轨道指针
                                cur_track    <= I_track_sel;
                                track_active <= sel_found;
                                font_left    <= 4'd0;
                                if (sel_found) begin
                                    data_addr <= sel_hdr;
                                    data_left <= sel_dsec;
                                    fetch_hdr  <= 1'b1;
                                end
                            end
                            else if (fetch_hdr) begin
                                // 头扇区抓完：有汉字则先读字库扇区，否则直接进 PCM
                                fetch_hdr <= 1'b0;
                                data_addr <= data_addr + 32'd1;
                                if (glyph_cnt_hold != 8'd0) begin
                                    // 字库扇区数 = ceil(汉字数*32/512) = ceil(汉字数/16)
                                    font_left <= (glyph_cnt_hold >> 4)
                                               + ((glyph_cnt_hold[3:0] != 4'd0) ? 4'd1 : 4'd0);
                                    font_wa   <= 11'd0;
                                    font_lsb  <= 8'd0;
                                end
                                else begin
                                    pcm_base_r <= data_addr + 32'd1;
                                    pcm_len_r  <= data_left;
                                    txt_toggle <= ~txt_toggle;
                                    txt_len_o  <= txt_len_hold;
                                end
                            end
                            else if (font_left != 4'd0) begin
                                // 字库扇区链：全部灌完才发布字幕，随后进入 PCM
                                data_addr <= data_addr + 32'd1;
                                data_left <= data_left - 32'd1;
                                if (font_left == 4'd1) begin
                                    font_left  <= 4'd0;
                                    pcm_base_r <= data_addr + 32'd1;
                                    pcm_len_r  <= data_left - 32'd1;
                                    txt_toggle <= ~txt_toggle;
                                    txt_len_o  <= txt_len_hold;
                                end
                                else begin
                                    font_left <= font_left - 4'd1;
                                end
                            end
                            else if (data_left != 32'd1) begin
                                data_left <= data_left - 32'd1;
                                data_addr <= data_addr + 32'd1;
                            end
                            else begin
                                // 本轨道播完一遍，循环：直接回 PCM 起始（不重读头/字库）
                                data_left <= pcm_len_r;
                                data_addr <= pcm_base_r;
                            end
                        end
                    end

                    ST_DONE: begin
                        O_state_code <= 4'd3;       // 卡上没有任何音频文件
                        sd_sec_read  <= 1'b0;
                    end

                    default: state <= ST_IDLE;
                endcase
            end
        end
    end

endmodule
