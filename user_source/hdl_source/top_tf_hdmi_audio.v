
module top(
    input                       clk,
    input                       rst_n,
    input                       key2,           // 手动下一张（板载丝印 KEY2；丝印 KEY1=A2 为复位 rst_n）
    input                       sw1,            // 自动轮播开关：高=自动，低=手动
    input                       key3,           // 菜单选择：每按一次计数器 +1（0对比度1亮度2音量3音调4转场特效5雪花，回绕）
    input                       sw2,            // 参数方向：高=往上，低=往下
    input                       sw3,            // 菜单滚动方向：低=K3 向下，高=K3 向上（2026-10-26 启用）
    input                       sw4,            // 音频可视化开关（高=显示，低=隐藏；提示在 bri_sen_osd）
    input                       key4,           // 参数调节：每按一次按 SW2 方向走一档（当前 K3 选中项；特效 0..4 循环）

    output [5:0]                seg_sel,
    output [7:0]                seg_data,

    // HDMI TMDS
    output                      HDMI_CLK_P,
    output                      HDMI_D2_P,
    output                      HDMI_D1_P,
    output                      HDMI_D0_P,

    // HDMI DDC
    output                      HDMI_DDC_SCL,
    inout                       HDMI_DDC_SDA,

    // TF card SPI
    output                      sd_ncs,
    output                      sd_dclk,
    output                      sd_mosi,
    input                       sd_miso
);

parameter MEM_DATA_BITS = 32;
parameter ADDR_BITS     = 21;
parameter BUSRT_BITS    = 10;
parameter FRAME_PIXELS  = 24'd307200;   // 640*480
parameter BUF0_ADDR     = 24'd0;
parameter BUF1_ADDR     = FRAME_PIXELS;

wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

wire sd_card_clk;
wire ext_mem_clk;
wire ext_mem_clk_sft;
wire video_clk;
wire hdmi_5x_clk;

wire hs;
wire vs;
wire de;

wire [23:0] vout_data_raw;
wire [23:0] vout_data;
wire        display_valid;

wire [3:0]  state_code;
wire [6:0]  seg_data_0;

wire        video_read_req;
wire        video_read_req_ack;
wire        video_read_en;
wire [31:0] video_read_data;

wire        sd_card_write_en;
wire [31:0] sd_card_write_data;
wire        sd_card_write_req;
wire        sd_card_write_req_ack;
wire        frame_write_finish;
reg         frame_write_toggle_mem;

wire [1:0]  write_buf_idx;
wire [1:0]  disp_buf_idx;

wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS-1:0] Sdr_rd_dout;
wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS-1:0] App_wr_din;
wire [3:0] App_wr_dm;

wire hs_0;
wire vs_0;
wire de_0;

// HDMI 1.4b 音频发射相关
wire        audio_pll_lock;
wire        audio_mclk;
wire        audio_i2s_bclk;
wire        audio_i2s_lrck;
wire        audio_i2s_dout;
wire        audio_valid;
wire [23:0] audio_left_data;
wire [23:0] audio_right_data;
wire        acr_valid;
wire [19:0] acr_cts;
wire [19:0] acr_n;

// TF 卡音频链路（SD 扇区读 → 双时钟 FIFO → I2S）
wire        sd_init_done;
wire        img_sd_sec_read;
wire [31:0] img_sd_sec_read_addr;
wire [7:0]  img_sd_sec_read_data;
wire        img_sd_sec_read_data_valid;
wire        img_sd_sec_read_end;
wire        aud_sd_sec_read;
wire [31:0] aud_sd_sec_read_addr;
wire [7:0]  aud_sd_sec_read_data;
wire        aud_sd_sec_read_data_valid;
wire        aud_sd_sec_read_end;
wire        wav_stream_mode;
wire        wav_urgent;
wire [3:0]  wav_state_code;    // 音频链路状态：0空闲 1扫描中 2播放中 3卡上无音频 4当前图片无对应音频
wire [6:0]  seg_data_audio;
wire [1:0]  disp_img_idx;      // 当前显示图片编号（音频跟随切换）
wire        img_commit_tgl;    // 图片提交 toggle（切图 → 分辨率 OSD）
wire [1:0]  img_tier;          // 当前显示图源分辨率挡 0/1/2
wire [2:0]  wav_found_count;   // 诊断：音频扫描找到的轨道数
wire [1:0]  wav_cur_track;     // 诊断：当前播放轨道
wire [6:0]  seg_data_txt;
wire [2:0]  dbg_code;       // 提交流程诊断码（sd_card_bmp 输出，暂不显示）
// 字幕链路（wav_reader 抓文字+汉字字库 → 双时钟 RAM → video 域渲染叠加）
wire        txt_we, txt_toggle;
wire [7:0]  txt_waddr, txt_wdata, txt_len_o;
wire [7:0]  txt_raddr, txt_rdata;
wire        font_we;
wire [10:0] font_waddr, cn_raddr;
wire [15:0] font_wdata, cn_dout;
wire        sub_vs, sub_de;
wire [23:0] sub_rgb;

// 图片切换过渡特效（K3 选择编号，与 sd_card_bmp 握手淡入淡出）
// 0无 1淡入淡出 2滑动 3栅格化 4线性擦除 5粒子溶解
// 档位由 bri_sen_osd 菜单第 5 行（K3 选中后 K4 调整）经 toggle 事件控制
wire [2:0]  effect_sel;
wire        fade_req, fade_ack;
wire        eff_up_tgl, eff_dn_tgl;  // 特效 ±1 事件（video → sd_card_clk，toggle 握手）
wire        trans_vs, trans_de;
wire [23:0] trans_rgb;
wire        wipe_en;           // 线性擦除进行中（video→mem 跨域）
wire [4:0]  wipe_gray;        // 擦除步进 0..20 格雷码（video→mem 跨域）

// 亮度/对比度/音量/音调调节 + 中文 OSD（SW4SW3 选择、SW2 方向、K4 走档）
wire        osd_vs, osd_de;
wire [23:0] osd_rgb;
wire [9:0]  vol_gain, play_rate;

// 音频可视化（FFT 频谱 → 屏幕底部点阵）
wire [191:0] spec_levels;
wire        viz_vs, viz_de;
wire [23:0] viz_rgb;
wire [6:0]  seg_data_found, seg_data_img, seg_data_cur;
wire [11:0] wav_fifo_usedw;
wire        wav_fifo_we;
wire [15:0] wav_fifo_di;
wire        wav_fifo_full;
wire        wav_fifo_re;
wire [15:0] wav_fifo_dout;
wire        wav_fifo_valid;
wire        wav_fifo_empty;

wire        axis_s_user;
wire        axis_s_valid;
wire        axis_s_last;
wire [23:0] axis_s_data;
wire        axis_s_ready;

wire        edid_trig;
wire        edid_valid;
wire [7:0]  edid_data;

wire [9:0]  tmds_ch0_data;
wire [9:0]  tmds_ch1_data;
wire [9:0]  tmds_ch2_data;
wire [9:0]  tmds_clk_data;

// 统一复位：TF 图像链路 + HDMI 音频链路
wire rst_all;
assign rst_all = ~rst_n;                  // 系统域只看板上复位
wire rst_aud = ~rst_n | ~audio_pll_lock;   // 仅音频 MCLK 域等 PLL 锁定

// 保持你原来的 TF / SDRAM / video 时钟
sys_pll sys_pll_m0(
    .refclk     (clk),
    .clk0_out   (sd_card_clk),
    .clk1_out   (ext_mem_clk),
    .clk2_out   (ext_mem_clk_sft),
    .reset      (1'b0)
);

video_pll video_pll_m0(
    .refclk     (clk),
    .clk0_out   (video_clk),
    .clk1_out   (hdmi_5x_clk),
    .reset      (1'b0)
);

// 复用你 I2S->HDMI 工程里的 PLL，只取 12.288MHz 音频主时钟
PLL_HDMI_AUDIO u_audio_pll(
    .refclk     (clk),
    .reset      (1'b0),
    .extlock    (audio_pll_lock),
    .clk0_out   (),
    .clk1_out   (),
    .clk2_out   (audio_mclk)
);

// mem_clk 域把 write_finish 单拍转成 toggle，供 sd_card_clk 域可靠同步
always @(posedge ext_mem_clk or posedge rst_all) begin
    if (rst_all)
        frame_write_toggle_mem <= 1'b0;
    else if (frame_write_finish)
        frame_write_toggle_mem <= ~frame_write_toggle_mem;
end

// ===================== TF 多图扫描与缓存（双缓冲） =====================
sd_card_bmp #(
    .CLK_FREQ_HZ       (100_000_000),
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd524287),   // 256MB：多轨音频在前，图片会被挤到 64MB 之外
    .SCAN_TARGET_COUNT (3'd4)
) sd_card_bmp_m0(
    .clk               (sd_card_clk),
    .rst               (rst_all),
    .key_next          (key2),
    .auto_sw           (sw1),
    .eff_up_tgl        (eff_up_tgl),
    .eff_dn_tgl        (eff_dn_tgl),
    .effect_sel        (effect_sel),
    .fade_req          (fade_req),
    .fade_ack          (fade_ack),
    .state_code        (state_code),
    .dbg_code          (dbg_code),
    .disp_img_idx      (disp_img_idx),
    .img_commit_tgl    (img_commit_tgl),   // 切图提交 → 右上角分辨率 OSD
    .img_tier          (img_tier),
    .bmp_width         (16'd640),
    .bmp_height        (16'd480),
    .display_valid     (display_valid),

    .write_finish_toggle(frame_write_toggle_mem),
    .write_buf_idx     (write_buf_idx),
    .disp_buf_idx      (disp_buf_idx),

    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data),

    // 逻辑扇区读接口（物理引脚由下方仲裁器统一管理）
    .sd_init_done      (sd_init_done),
    .sd_sec_read       (img_sd_sec_read),
    .sd_sec_read_addr  (img_sd_sec_read_addr),
    .sd_sec_read_data  (img_sd_sec_read_data),
    .sd_sec_read_data_valid (img_sd_sec_read_data_valid),
    .sd_sec_read_end   (img_sd_sec_read_end)
);

// ===================== SD 读通道仲裁：图片 + 音频共享一张 TF 卡 =====================
sd_read_arbiter u_sd_read_arbiter(
    .clk                        (sd_card_clk),
    .rst                        (rst_all),
    .img_sd_sec_read            (img_sd_sec_read),
    .img_sd_sec_read_addr       (img_sd_sec_read_addr),
    .img_sd_sec_read_data       (img_sd_sec_read_data),
    .img_sd_sec_read_data_valid (img_sd_sec_read_data_valid),
    .img_sd_sec_read_end        (img_sd_sec_read_end),
    .aud_sd_sec_read            (aud_sd_sec_read),
    .aud_sd_sec_read_addr       (aud_sd_sec_read_addr),
    .aud_urgent                 (wav_urgent),
    .aud_sd_sec_read_data       (aud_sd_sec_read_data),
    .aud_sd_sec_read_data_valid (aud_sd_sec_read_data_valid),
    .aud_sd_sec_read_end        (aud_sd_sec_read_end),
    .sd_init_done               (sd_init_done),
    .SD_nCS                     (sd_ncs),
    .SD_DCLK                    (sd_dclk),
    .SD_MOSI                    (sd_mosi),
    .SD_MISO                    (sd_miso)
);

seg_decoder seg_decoder_m0(
    .bin_data          (state_code),
    .seg_data          (seg_data_0)
);

// 音频链路状态显示（占用数码管空闲位）
seg_decoder seg_decoder_audio(
    .bin_data          (wav_state_code),
    .seg_data          (seg_data_audio)
);

// 诊断：找到轨道数 / 当前图片号 / 当前播放轨道
seg_decoder seg_decoder_found(
    .bin_data          ({1'b0, wav_found_count}),
    .seg_data          (seg_data_found)
);
seg_decoder seg_decoder_imgidx(
    .bin_data          ({2'b00, disp_img_idx}),
    .seg_data          (seg_data_img)
);
seg_decoder seg_decoder_curtrk(
    .bin_data          ({2'b00, wav_cur_track}),
    .seg_data          (seg_data_cur)
);

// 当前特效编号显示（菜单选中"转场特效"后 K4 调整：0无..4擦除 5粒子，OSD 第 5 行同显示）
seg_decoder seg_decoder_eff(
    .bin_data          ({1'b0, effect_sel}),
    .seg_data          (seg_data_txt)
);
seg_scan seg_scan_m0(
    .clk               (clk),
    .rst_n             (rst_n),
    .seg_sel           (seg_sel),
    .seg_data          (seg_data),
    .seg_data_0        ({1'b1,seg_data_found}),
    .seg_data_1        ({1'b1,seg_data_img}),
    .seg_data_2        ({1'b1,seg_data_cur}),
    .seg_data_3        ({1'b1,seg_data_txt}),
    .seg_data_4        ({1'b1,seg_data_audio}),
    .seg_data_5        ({1'b1,seg_data_0})
);

// ===================== 原图像时序与帧缓存 =====================
video_timing_data video_timing_data_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0)
);

video_delay video_delay_m0(
    .video_clk         (video_clk),
    .rst               (rst_all),
    .read_en           (video_read_en),
    .read_data         (video_read_data[31:8]),
    .hs                (hs_0),
    .vs                (vs_0),
    .de                (de_0),
    .hs_r              (hs),
    .vs_r              (vs),
    .de_r              (de),
    .vout_data         (vout_data_raw)
);

// 首图提交前黑屏；提交后一直显示当前显示缓冲区内容
assign vout_data = display_valid ? vout_data_raw : 24'd0;

frame_read_write #(
    .WRITE_V_FLIP     (1),
    .FRAME_WIDTH      (640),
    .FRAME_HEIGHT     (480)
) frame_read_write_m0(
    .mem_clk           (ext_mem_clk),
    .rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),

    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout),

    .read_clk          (video_clk),
    .read_req          (video_read_req),
    .read_req_ack      (video_read_req_ack),
    .read_finish       (),
    .read_addr_0       (BUF0_ADDR),
    .read_addr_1       (BUF1_ADDR),
    .read_addr_2       (24'd0),
    .read_addr_3       (24'd0),
    .read_addr_index   (disp_buf_idx),
    .read_len          (FRAME_PIXELS),
    .read_wipe_en      (wipe_en),        // 线性擦除：读地址按边界在双缓冲间切换
    .read_wipe_gray    (wipe_gray),
    .read_wipe_index   (write_buf_idx),  // 擦除期间新图所在的缓冲（加载期间恒定）
    .read_en           (video_read_en),
    .read_data         (video_read_data),

    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_din        (App_wr_din),
    .App_wr_dm         (App_wr_dm),

    .write_clk         (sd_card_clk),
    .write_req         (sd_card_write_req),
    .write_req_ack     (sd_card_write_req_ack),
    .write_finish      (frame_write_finish),
    .write_addr_0      (BUF0_ADDR),
    .write_addr_1      (BUF1_ADDR),
    .write_addr_2      (24'd0),
    .write_addr_3      (24'd0),
    .write_addr_index  (write_buf_idx),
    .write_len         (FRAME_PIXELS),
    .write_en          (sd_card_write_en),
    .write_data        (sd_card_write_data)
);

sdram U3(
    .Clk               (ext_mem_clk),
    .Clk_sft           (ext_mem_clk_sft),
    .Rst               (rst_all),
    .Sdr_init_done     (Sdr_init_done),
    .Sdr_init_ref_vld  (Sdr_init_ref_vld),
    .Sdr_busy          (Sdr_busy),
    .App_wr_en         (App_wr_en),
    .App_wr_addr       (App_wr_addr),
    .App_wr_dm         (App_wr_dm),
    .App_wr_din        (App_wr_din),
    .App_rd_en         (App_rd_en),
    .App_rd_addr       (App_rd_addr),
    .Sdr_rd_en         (Sdr_rd_en),
    .Sdr_rd_dout       (Sdr_rd_dout)
);

// ===================== 音频：TF 卡 WAV → I2S（64fs）→ HDMI =====================
// 读卡扫描 + 水位预取（sd_card_clk 域）
sd_wav_reader #(
    .SCAN_START_SECTOR (32'd0),
    .SCAN_MAX_SECTOR   (32'd524287)   // 256MB：与图片扫描同范围，覆盖 4 轨大音频的头部位置
) u_sd_wav_reader(
    .clk                    (sd_card_clk),
    .rst                    (rst_all),
    .sd_init_done           (sd_init_done),
    .I_track_sel            (disp_img_idx),   // 音频跟随显示图片切换
    .sd_sec_read            (aud_sd_sec_read),
    .sd_sec_read_addr       (aud_sd_sec_read_addr),
    .sd_sec_read_data       (aud_sd_sec_read_data),
    .sd_sec_read_data_valid (aud_sd_sec_read_data_valid),
    .sd_sec_read_end        (aud_sd_sec_read_end),
    .fifo_usedw             (wav_fifo_usedw),
    .fifo_full              (wav_fifo_full),
    .fifo_we                (wav_fifo_we),
    .fifo_di                (wav_fifo_di),
    .O_stream_mode          (wav_stream_mode),
    .O_urgent               (wav_urgent),
    .O_state_code           (wav_state_code),
    .O_found_count          (wav_found_count),
    .O_cur_track            (wav_cur_track),
    .txt_we                 (txt_we),
    .txt_waddr              (txt_waddr),
    .txt_wdata              (txt_wdata),
    .txt_toggle             (txt_toggle),
    .txt_len_o              (txt_len_o),
    .font_we                (font_we),
    .font_waddr             (font_waddr),
    .font_wdata             (font_wdata)
);

// 字幕文字 RAM：写侧 sd_card_clk（读卡抓取），读侧 video_clk（逐像素取字符）
text_ram_dp u_text_ram(
    .clkw   (sd_card_clk),
    .we     (txt_we),
    .waddr  (txt_waddr),
    .wdata  (txt_wdata),
    .clkr   (video_clk),
    .raddr  (txt_raddr),
    .dout   (txt_rdata)
);

// 汉字子集字库 RAM：写侧 sd_card_clk（wav_reader 灌字库扇区），
// 读侧 video_clk（subtitle_overlay 逐像素取 16x16 行点阵）
font_cn_ram u_font_cn_ram(
    .clkw   (sd_card_clk),
    .we     (font_we),
    .waddr  (font_waddr),
    .wdata  (font_wdata),
    .clkr   (video_clk),
    .raddr  (cn_raddr),
    .dout   (cn_dout)
);

// 双时钟 FIFO：写侧 100MHz 读卡域，读侧 12.288MHz 音频域
audio_fifo_dc u_audio_fifo(
    .rst      (rst_aud),
    .clkw     (sd_card_clk),
    .we       (wav_fifo_we),
    .di       (wav_fifo_di),
    .full     (wav_fifo_full),
    .wrusedw  (wav_fifo_usedw),
    .clkr     (audio_mclk),
    .re       (wav_fifo_re),
    .dout     (wav_fifo_dout),
    .valid_r  (wav_fifo_valid),
    .empty    (wav_fifo_empty)
);

// I2S 发送：时序与原测试音模块一致，样本来自 FIFO（16bit→24bit 左对齐）
// 带音量缩放与倍速播放（参数来自 OSD 模块，video 域准静态）
sd_wav_i2s_player u_sd_wav_i2s_player(
    .I_mclk       (audio_mclk),
    .I_rst        (rst_aud),
    .I_fifo_empty (wav_fifo_empty),
    .O_fifo_re    (wav_fifo_re),
    .I_fifo_dout  (wav_fifo_dout),
    .I_fifo_valid (wav_fifo_valid),
    .I_vol_gain   (vol_gain),
    .I_rate       (play_rate),
    .O_i2s_BCLK   (audio_i2s_bclk),
    .O_i2s_LRCK   (audio_i2s_lrck),
    .O_i2s_DOUT   (audio_i2s_dout)
);

I2S_receiver u_I2S_receiver(
    .I_clk              (video_clk),
    .I_rst              (rst_all),
    .I_i2s_BCLK         (audio_i2s_bclk),
    .I_i2s_LRCK         (audio_i2s_lrck),
    .I_i2s_DOUT         (audio_i2s_dout),
    .O_audio_valid      (audio_valid),
    .O_audio_left_data  (audio_left_data),
    .O_audio_right_data (audio_right_data)
);

// Goertzel 8 频段频谱（video_clk 域，吃 I2S 接收器的左声道样本）
spectrum_core u_spectrum_core(
    .I_clk          (video_clk),
    .I_rst          (rst_all),
    .I_sample_valid (audio_valid),
    .I_sample_left  (audio_left_data),
    .O_levels       (spec_levels)
);

audio_arc_calculate #(
    .ACR_N         (6144)
) u_audio_arc_calculate (
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_audio_valid (audio_valid),
    .O_acr_valid   (acr_valid),
    .O_acr_cts     (acr_cts),
    .O_acr_n       (acr_n)
);

// ===================== RGB/DE 转 AXIS 视频 =====================
// 下雪氛围特效（2026-10-24 图层调整：置于频谱/字幕/转场/OSD 之下——
// 雪花从字幕和频谱条"后面"飘过，转场特效也会作用于雪）
// K3 菜单第 6 行"雪花:关/小/盛"（K4 切换），档位同域送 snow_overlay
wire [1:0]  snow_lvl;               // 粒子特效档：0无 / 2雪花(64 片) / 3星空(40 星)
wire        viz_show;               // SW4 频谱可视化开关（bri_sen_osd 同步输出电平）
wire        snow_vs, snow_de;
wire [23:0] snow_rgb;

snow_overlay u_snow_overlay(
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .I_vs    (vs),
    .I_de    (de),
    .I_rgb   (vout_data),
    .I_lvl   (snow_lvl),
    .O_vs    (snow_vs),
    .O_de    (snow_de),
    .O_rgb   (snow_rgb)
);

// 底部滚动字幕叠加（中英混排 16px 单元，video_clk 域，随当前图片/轨道切换）
// 四周频谱可视化（纯组合叠加，零流水线延迟）
viz_overlay u_viz_overlay(
    .I_clk    (video_clk),
    .I_rst    (rst_all),
    .I_vs     (snow_vs),
    .I_de     (snow_de),
    .I_rgb    (snow_rgb),
    .I_levels (spec_levels),
    .I_show   (viz_show),
    .O_vs     (viz_vs),
    .O_de     (viz_de),
    .O_rgb    (viz_rgb)
);

subtitle_overlay u_subtitle_overlay(
    .I_clk       (video_clk),
    .I_rst       (rst_all),
    .I_vs        (viz_vs),
    .I_de        (viz_de),
    .I_rgb       (viz_rgb),
    .O_ram_raddr (txt_raddr),
    .I_ram_dout  (txt_rdata),
    .O_cn_raddr  (cn_raddr),
    .I_cn_dout   (cn_dout),
    .I_txt_toggle(txt_toggle),
    .I_txt_len   (txt_len_o),
    .O_vs        (sub_vs),
    .O_de        (sub_de),
    .O_rgb       (sub_rgb)
);

// 图片切换过渡特效（K3 选择：0无 1黑场淡入淡出 2滑动 3栅格化 4线性擦除）
// 线性擦除的混合由帧缓存读侧完成：frame_read_write 按 wipe 边界在
// 显示缓冲（旧图）与写入缓冲（新图）之间逐字切换读地址
img_transition u_img_transition(
    .I_clk      (video_clk),
    .I_rst      (rst_all),
    .I_vs       (sub_vs),
    .I_de       (sub_de),
    .I_rgb      (sub_rgb),
    .I_effect   (effect_sel),
    .I_fade_req (fade_req),
    .O_fade_ack (fade_ack),
    .O_wipe_en  (wipe_en),
    .O_wipe_gray(wipe_gray),
    .O_vs       (trans_vs),
    .O_de       (trans_de),
    .O_rgb      (trans_rgb)
);

// 亮度/对比度/音量/音调/转场特效/雪花六项菜单 + 中文 OSD（链路末端：画面调节作用于全画面，OSD 不受影响）
// K3 菜单选择（0对比度 1亮度 2音量 3音调 4转场特效 5雪花）、SW2 方向、K4 走档；
// 转场特效档位经 toggle 事件送往 sd_card_bmp 的 effect_cnt；
// 雪花档位送 snow_overlay（同域准静态，实例在链路上游 viz_overlay 之前）

bri_sen_osd u_bri_sen_osd(
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .I_vs    (trans_vs),
    .I_de    (trans_de),
    .I_rgb   (trans_rgb),
    .I_sw1   (sw1),
    .I_sw2   (sw2),
    .I_sw3   (sw3),
    .I_sw4   (sw4),
    .I_k4    (key4),
    .I_k3    (key3),
    .I_img_tgl  (img_commit_tgl),
    .I_img_tier (img_tier),
    .O_vol_gain (vol_gain),
    .O_rate     (play_rate),
    .O_eff_up   (eff_up_tgl),
    .O_eff_dn   (eff_dn_tgl),
    .O_snow_lvl (snow_lvl),
    .O_viz_show (viz_show),
    .O_vs    (osd_vs),
    .O_de    (osd_de),
    .O_rgb   (osd_rgb)
);

video_rgb_to_axis_640x480 u_video_rgb_to_axis_640x480(
    .I_clk         (video_clk),
    .I_rst         (rst_all),
    .I_vs          (osd_vs),
    .I_de          (osd_de),
    .I_rgb         (osd_rgb),
    .O_video_user  (axis_s_user),
    .O_video_valid (axis_s_valid),
    .O_video_last  (axis_s_last),
    .O_video_data  (axis_s_data)
);

// 上电后自动打一拍，触发一次 EDID 读取
startup_pulse #(
    .CNT_MAX(20'd100000)
) u_startup_pulse (
    .I_clk   (video_clk),
    .I_rst   (rst_all),
    .O_pulse (edid_trig)
);

// ===================== 带音频的 HDMI 1.4b 发射 =====================
hdmi_1_4b_transmitter_core_wrapper #(
    .DEVICE                 ( "EG"       ),
    .HTOTAL                 ( 800        ),
    .HSA                    ( 96         ),
    .HFP                    ( 16         ),
    .HBP                    ( 48         ),
    .HACTIVE                ( 640        ),
    .VTOTAL                 ( 525        ),
    .VSA                    ( 2          ),
    .VFP                    ( 10         ),
    .VBP                    ( 33         ),
    .VACTIVE                ( 480        ),
    .VIDEO_VIC              ( 1          ),
    .VIDEO_TPG              ( "Disable"  ),
    .VIDEO_FORMAT           ( "RGB"      ),
    .AUDIO_SAMPLE_RATE      ( "48K"      ),
    .IIC_SCL_DIV            ( 250        )
) u_hdmi_1_4b_transmitter_core_wrapper(
    .I_pixel_clk        (video_clk),
    .I_rst              (rst_all),
    .I_edid_read_trig   (edid_trig),
    .O_edid_read_valid  (edid_valid),
    .O_edid_read_data   (edid_data),

    .I_axis_s_user      (axis_s_user),
    .I_axis_s_valid     (axis_s_valid),
    .I_axis_s_last      (axis_s_last),
    .I_axis_s_data      (axis_s_data),
    .O_axis_s_ready     (axis_s_ready),

    .I_audio_valid      (audio_valid),
    .I_audio_left_data  (audio_left_data),
    .I_audio_right_data (audio_right_data),
    .I_acr_valid        (acr_valid),
    .I_acr_cts          (acr_cts),
    .I_acr_n            (acr_n),

    .O_video_locked     (),
    .O_ddc_scl          (HDMI_DDC_SCL),
    .IO_ddc_sda         (HDMI_DDC_SDA),

    .O_ch0_tmds_data    (tmds_ch0_data),
    .O_ch1_tmds_data    (tmds_ch1_data),
    .O_ch2_tmds_data    (tmds_ch2_data),
    .O_clk_tmds_data    (tmds_clk_data)
);

hdmi_phy_wrapper #(
    .DEVICE ( "EG" )
) u_hdmi2phy_wrapper(
    .I_pixel_clk        (video_clk),
    .I_serial_clk       (hdmi_5x_clk),
    .I_rst              (rst_all),
    .I_tmds_channel_0   (tmds_ch0_data),
    .I_tmds_channel_1   (tmds_ch1_data),
    .I_tmds_channel_2   (tmds_ch2_data),
    .I_tmds_channel_clk (tmds_clk_data),
    .O_tmds_ch0_p       (HDMI_D0_P),
    .O_tmds_ch1_p       (HDMI_D1_P),
    .O_tmds_ch2_p       (HDMI_D2_P),
    .O_tmds_clk_p       (HDMI_CLK_P)
);

endmodule
