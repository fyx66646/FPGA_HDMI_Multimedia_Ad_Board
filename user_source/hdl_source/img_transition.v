// ============================================================================
// img_transition : 图片切换过渡特效（video_clk 域，零流水线延迟）
// 与 sd_card_bmp 配合实现"旧图淡出 → 过渡 → 新图进入"：
//   1. sd_card_bmp 在新图写好后发 fade_req（toggle）并挂起提交；
//   2. 本模块把当前画面（旧图）逐步过渡（淡出/滑出），完成时回 fade_ack；
//   3. sd_card_bmp 收到 ack 后提交新图（显示缓冲区切换）；
//   4. 本模块再播放"进入"动画（淡入/滑入）。
//
// 特效编号（I_effect，异步输入内部两级同步）：
//   0 = 无特效（正常直通，本模块不会被请求）
//   1 = 黑场淡入淡出：淡出 16 帧 + 淡入 16 帧（约 0.53s）
//   2 = 滑动切换：旧图被黑幕从右向左收走（20 帧）→ 新图从右向左滑入（20 帧）
//   3 = 栅格化（棋盘间隔）：旧图一半格子先熄灭成棋盘格（8 帧）→ 全黑 →
//       提交新图 → 新图一半格子先点亮成棋盘格（8 帧）→ 全亮（8 帧）
//   4 = 线性擦除：新旧两图直接混合显示，垂直边界从右向左扫过（21 帧），
//       边界右侧显示新图、左侧显示旧图，全程无黑场；擦完才回 ack 提交。
//       混合由帧缓存读侧完成（frame_fifo_read 按 wipe 边界在两个缓冲区间
//       切换读地址），本模块只输出擦除进度（灰码步进 0..20 跨域）。
//   5 = 粒子溶解：画面按 4x4 块随机散布熄灭（16 帧）→ 全黑 → 提交 →
//       新图按同样的散布顺序浮现（16 帧）。每块的消失时刻由块坐标的
//       二级混合哈希现场算出（零存储）；哈希公式经 Python 预览验证
//       无行/列/对角伪影（_particle_check2.py）。
// 跨域信号均为 toggle 电平（I_fade_req / O_fade_ack），接收侧边沿检测。
// ============================================================================
module img_transition (
    input  wire        I_clk,          // video_clk 25MHz
    input  wire        I_rst,          // 高有效

    input  wire        I_vs,
    input  wire        I_de,
    input  wire [23:0] I_rgb,

    input  wire [2:0]  I_effect,       // 特效编号（sd_card_clk 域）
    input  wire        I_fade_req,     // 淡出请求 toggle（sd_card_clk 域）
    output reg         O_fade_ack,     // 淡出完成 toggle（video_clk 域）

    output wire        O_wipe_en,      // 线性擦除进行中（跨域到 mem_clk，2FF 同步）
    output wire [4:0]  O_wipe_gray,    // 擦除步进 0..20 的格雷码（步进每帧 +1，跨域安全）

    output wire        O_vs,
    output wire        O_de,
    output reg  [23:0] O_rgb
);

    localparam [3:0] S_IDLE    = 4'd0;
    localparam [3:0] S_FADEOUT = 4'd1;
    localparam [3:0] S_HOLD    = 4'd2;
    localparam [3:0] S_FADEIN  = 4'd3;
    localparam [3:0] S_SLIDE   = 4'd4;   // 新图从右向左滑入
    localparam [3:0] S_GRID    = 4'd5;   // 新图栅格逐格点亮
    localparam [3:0] S_SLIDEOUT= 4'd6;   // 旧图被黑幕从右向左收走
    localparam [3:0] S_GRIDOUT = 4'd7;   // 旧图栅格逐格熄灭
    localparam [3:0] S_WIPE    = 4'd8;   // 线性擦除（读侧双缓冲混合）
    localparam [3:0] S_PARTOUT = 4'd9;   // 粒子溶解：旧图按哈希散布熄灭
    localparam [3:0] S_PART    = 4'd10;  // 粒子溶解：新图按同顺序浮现

    // ---------------- 跨域同步（特效编号 / 淡出请求） ----------------
    reg [2:0] eff_s0, eff_s1;
    reg [2:0] req_sync;

    reg [3:0]  state;
    reg [7:0]  mix;          // 255=正常画面，0=全黑
    reg [1:0]  hold_cnt;
    reg        vs_d;
    reg [10:0] x_cnt;        // 当前像素横坐标（与 I_de/I_rgb 同拍）
    reg [10:0] y_cnt;        // 当前像素纵坐标
    reg [9:0]  slide_x;      // 滑动边界：>=slide_x 显示新图，其余黑
    reg [5:0]  grid_t;       // 栅格相位：0..7 棋盘格（半数格子），8..15 全部
    reg [3:0]  part_f;       // 粒子溶解帧计数 0..15（块阈值 4 位）
    reg [4:0]  wipe_step;    // 擦除步进 0..20：边界 x = 640 - step*32
    reg        wipe_en_r;    // 擦除进行中（打拍输出，跨域单比特）
    reg [4:0]  wipe_gray_r;  // 擦除步进格雷码（打拍输出，跨域多位总线用灰码）

    wire req_edge   = (req_sync[2] != req_sync[1]);
    wire frame_edge = (vs_d != I_vs);

    // 特效参数（组合）
    wire [7:0] step_out = (eff_s1 == 3'd1) ? 8'd16 : 8'd32;   // 特效1 16帧淡出；2/3 快速 8 帧
    wire [7:0] step_in  = 8'd16;                             // 特效1淡入 16 帧

    // ---------------- 粒子溶解：4x4 块坐标的二级混合哈希（组合，零存储） ----------------
    // acc=(bx*271+by*137)[14:0]; acc^=acc>>7; acc=(acc*61)[15:0]; acc^=acc>>5; 阈值=acc[3:0]
    // Python 预览验证：列存活率 0.36~0.55（随机 ±3σ 内）、邻块相同率 ~0.05、直方均匀
    wire [7:0]  p_bx   = x_cnt[9:2];                          // 0..159
    wire [6:0]  p_by   = y_cnt[8:2];                          // 0..119
    wire [15:0] p_acc0 = {8'd0, p_bx} * 16'd271 + {9'd0, p_by} * 16'd137;
    wire [14:0] p_acc1 = p_acc0[14:0] ^ {7'd0, p_acc0[14:7]};
    wire [20:0] p_mul  = {6'd0, p_acc1} * 21'd61;
    wire [15:0] p_acc2 = p_mul[15:0] ^ {5'd0, p_mul[15:5]};
    wire [3:0]  p_thr  = p_acc2[3:0];                         // 块消失/浮现时刻 0..15

    // ---------------- 混合/滑动输出（组合） ----------------
    // 亮度缩放（淡出/保持阶段，目标为黑）
    wire [15:0] r_lo = {8'd0, I_rgb[23:16]} * {8'd0, mix};
    wire [7:0]  r_out = (r_lo + 16'd128) >> 8;
    wire [15:0] g_lo = {8'd0, I_rgb[15:8]}  * {8'd0, mix};
    wire [7:0]  g_out = (g_lo + 16'd128) >> 8;
    wire [15:0] b_lo = {8'd0, I_rgb[7:0]}   * {8'd0, mix};
    wire [7:0]  b_out = (b_lo + 16'd128) >> 8;

    always @(*) begin
        if (state == S_IDLE || state == S_WIPE)
            O_rgb = I_rgb;                      // 正常直通；擦除的混合在读侧已完成
        else if (state == S_SLIDE) begin
            // 新图滑入：x >= slide_x 区域已滑入
            if (x_cnt >= slide_x)
                O_rgb = I_rgb;
            else
                O_rgb = 24'd0;
        end
        else if (state == S_SLIDEOUT) begin
            // 旧图收走：x < slide_x 区域仍可见，其余被黑幕覆盖
            if (x_cnt < slide_x)
                O_rgb = I_rgb;
            else
                O_rgb = 24'd0;
        end
        else if (state == S_GRID) begin
            // 棋盘间隔点亮：前 8 帧偶格（cx^cy=0）先亮，后 8 帧全亮
            if ((grid_t >= 6'd8) || (x_cnt[5] ^ y_cnt[5]) == 1'b0)
                O_rgb = I_rgb;
            else
                O_rgb = 24'd0;
        end
        else if (state == S_GRIDOUT) begin
            // 棋盘间隔熄灭：前 8 帧偶格先灭（剩棋盘格），后 8 帧全灭
            if ((grid_t < 6'd8) && (x_cnt[5] ^ y_cnt[5]))
                O_rgb = I_rgb;
            else
                O_rgb = 24'd0;
        end
        else if (state == S_PARTOUT) begin
            // 粒子散灭：块阈值 p_thr，帧计数到达后熄灭（可见 = part_f < 阈值）
            if (part_f < p_thr)
                O_rgb = I_rgb;
            else
                O_rgb = 24'd0;
        end
        else if (state == S_PART) begin
            // 粒子浮现：同阈值同顺序，可见 = part_f >= 阈值
            if (part_f >= p_thr)
                O_rgb = I_rgb;
            else
                O_rgb = 24'd0;
        end
        else
            O_rgb = {r_out, g_out, b_out};      // 淡出/保持阶段按 mix 压暗
    end

    assign O_vs = I_vs;
    assign O_de = I_de;

    // 擦除控制输出（打拍后跨域；步进每帧才变一次，灰码保证采到旧值或新值）
    assign O_wipe_en   = wipe_en_r;
    assign O_wipe_gray = wipe_gray_r;

    // ---------------- 像素计数（与输入同拍） ----------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            x_cnt <= 11'd0;
            y_cnt <= 11'd0;
        end
        else begin
            if (I_de)
                x_cnt <= (x_cnt == 11'd639) ? 11'd0 : (x_cnt + 11'd1);
            else
                x_cnt <= 11'd0;

            if (vs_d != I_vs)
                y_cnt <= 11'd0;
            else if (I_de && (x_cnt == 11'd639))
                y_cnt <= y_cnt + 11'd1;
        end
    end

    // ---------------- 主状态机 ----------------
    always @(posedge I_clk or posedge I_rst) begin
        if (I_rst) begin
            eff_s0    <= 3'd0;
            eff_s1    <= 3'd0;
            req_sync  <= 3'b000;
            state     <= S_IDLE;
            mix       <= 8'd255;
            hold_cnt  <= 2'd0;
            vs_d      <= 1'b0;
            slide_x   <= 10'd640;
            grid_t    <= 6'd0;
            part_f    <= 4'd0;
            wipe_step <= 5'd0;
            wipe_en_r <= 1'b0;
            wipe_gray_r <= 5'd0;
            O_fade_ack<= 1'b0;
        end
        else begin
            eff_s0   <= I_effect;
            eff_s1   <= eff_s0;
            req_sync <= {req_sync[1:0], I_fade_req};
            vs_d     <= I_vs;

            // 擦除控制寄存器输出（与状态机同拍更新）
            wipe_en_r   <= (state == S_WIPE);
            wipe_gray_r <= (wipe_step >> 1) ^ wipe_step;

            case (state)
                S_IDLE: begin
                    mix <= 8'd255;
                    if (req_edge) begin
                        case (eff_s1)
                            3'd2: begin
                                slide_x <= 10'd640;    // 旧图从右向左被收走
                                state   <= S_SLIDEOUT;
                            end
                            3'd3: begin
                                grid_t <= 6'd0;        // 旧图栅格逐格熄灭
                                state  <= S_GRIDOUT;
                            end
                            3'd4: begin
                                wipe_step <= 5'd0;     // 线性擦除：边界 640，全旧图
                                state     <= S_WIPE;
                            end
                            3'd5: begin
                                part_f <= 4'd0;         // 粒子溶解：从全旧图开始散灭
                                state  <= S_PARTOUT;
                            end
                            3'd1: begin
                                state <= S_FADEOUT;
                            end
                            default: begin
                                // 无特效却收到请求（不应发生）：立即应答
                                O_fade_ack <= ~O_fade_ack;
                            end
                        endcase
                    end
                end

                S_FADEOUT: begin
                    if (req_edge) begin
                        // 过渡中收到新请求：立即应答并放弃当前过渡，防止握手丢失
                        O_fade_ack <= ~O_fade_ack;
                        state      <= S_IDLE;
                        mix        <= 8'd255;
                    end
                    else if (frame_edge) begin
                        if (mix <= step_out) begin
                            mix        <= 8'd0;
                            O_fade_ack <= ~O_fade_ack;   // 通知 sd 域：可以提交新图了
                            state      <= S_HOLD;
                            hold_cnt   <= 2'd0;
                        end
                        else begin
                            mix <= mix - step_out;
                        end
                    end
                end

                S_HOLD: begin
                    // 黑场保持 4 帧，覆盖 sd 域的同步 + 提交时延
                    if (req_edge) begin
                        O_fade_ack <= ~O_fade_ack;
                        state      <= S_IDLE;
                        mix        <= 8'd255;
                    end
                    else if (frame_edge) begin
                        if (hold_cnt == 2'd3) begin
                            case (eff_s1)
                                3'd2: begin
                                    slide_x <= 10'd640;     // 新图从右缘开始滑入
                                    state   <= S_SLIDE;
                                end
                                3'd3: begin
                                    grid_t <= 6'd0;         // 栅格揭示从头开始
                                    state  <= S_GRID;
                                end
                                3'd5: begin
                                    part_f <= 4'd0;         // 粒子浮现从头开始
                                    state  <= S_PART;
                                end
                                default: begin
                                    state <= S_FADEIN;
                                end
                            endcase
                        end
                        else begin
                            hold_cnt <= hold_cnt + 2'd1;
                        end
                    end
                end

                S_FADEIN: begin
                    if (req_edge) begin
                        O_fade_ack <= ~O_fade_ack;
                        state      <= S_IDLE;
                        mix        <= 8'd255;
                    end
                    else if (frame_edge) begin
                        // 注意：8 位加法会溢出回绕，必须先比较再加
                        if (mix >= (8'd255 - step_in)) begin
                            mix   <= 8'd255;
                            state <= S_IDLE;
                        end
                        else begin
                            mix <= mix + step_in;
                        end
                    end
                end

                S_SLIDE: begin
                    if (req_edge) begin
                        O_fade_ack <= ~O_fade_ack;
                        state      <= S_IDLE;
                        mix        <= 8'd255;
                    end
                    else if (frame_edge) begin
                        if (slide_x <= 10'd32) begin
                            slide_x <= 10'd0;
                            state   <= S_IDLE;          // 完全滑入，恢复直通
                        end
                        else begin
                            slide_x <= slide_x - 10'd32;   // 每帧左移 32px（20 帧滑完）
                        end
                    end
                end

                S_GRID: begin
                    if (req_edge) begin
                        O_fade_ack <= ~O_fade_ack;
                        state      <= S_IDLE;
                        mix        <= 8'd255;
                    end
                    else if (frame_edge) begin
                        if (grid_t >= 6'd15) begin
                            state <= S_IDLE;            // 全部格子点亮
                        end
                        else begin
                            grid_t <= grid_t + 6'd1;    // 8+8 帧：半格→全格
                        end
                    end
                end

                S_SLIDEOUT: begin
                    if (req_edge) begin
                        O_fade_ack <= ~O_fade_ack;
                        state      <= S_IDLE;
                        mix        <= 8'd255;
                    end
                    else if (frame_edge) begin
                        if (slide_x <= 10'd32) begin
                            slide_x    <= 10'd0;        // 旧图完全收走（全黑）
                            mix        <= 8'd0;         // HOLD 阶段保持全黑（防闪）
                            O_fade_ack <= ~O_fade_ack;  // 通知 sd 域提交新图
                            state      <= S_HOLD;
                            hold_cnt   <= 2'd0;
                        end
                        else begin
                            slide_x <= slide_x - 10'd32;   // 每帧左移 32px
                        end
                    end
                end

                S_GRIDOUT: begin
                    if (req_edge) begin
                        O_fade_ack <= ~O_fade_ack;
                        state      <= S_IDLE;
                        mix        <= 8'd255;
                    end
                    else if (frame_edge) begin
                        if (grid_t >= 6'd15) begin
                            mix        <= 8'd0;         // HOLD 阶段保持全黑（防闪）
                            O_fade_ack <= ~O_fade_ack;  // 旧图格子全熄（全黑）
                            state      <= S_HOLD;
                            hold_cnt   <= 2'd0;
                        end
                        else begin
                            grid_t <= grid_t + 6'd1;    // 8+8 帧：半格→全格
                        end
                    end
                end

                S_WIPE: begin
                    // 线性擦除：混合在读侧（frame_fifo_read 双缓冲地址切换），
                    // 本状态只推进边界步进；擦完（步进 20，全屏新图）才 ack 提交
                    if (req_edge) begin
                        O_fade_ack <= ~O_fade_ack;     // 新请求：立即应答放弃本次
                        state      <= S_IDLE;
                        wipe_step  <= 5'd0;
                    end
                    else if (frame_edge) begin
                        if (wipe_step >= 5'd20) begin
                            O_fade_ack <= ~O_fade_ack; // 擦除完成，通知 sd 域提交
                            state      <= S_IDLE;
                            wipe_step  <= 5'd0;
                        end
                        else begin
                            wipe_step <= wipe_step + 5'd1;  // 每帧边界左移 32px
                        end
                    end
                end

                S_PARTOUT: begin
                    // 粒子散灭：帧计数推进，块按各自哈希阈值陆续熄灭；
                    // 全灭（part_f=15）后 ack 提交，HOLD 保持全黑
                    if (req_edge) begin
                        O_fade_ack <= ~O_fade_ack;
                        state      <= S_IDLE;
                        mix        <= 8'd255;
                    end
                    else if (frame_edge) begin
                        if (part_f >= 4'd15) begin
                            mix        <= 8'd0;         // HOLD 阶段保持全黑（防闪）
                            O_fade_ack <= ~O_fade_ack;
                            state      <= S_HOLD;
                            hold_cnt   <= 2'd0;
                        end
                        else begin
                            part_f <= part_f + 4'd1;
                        end
                    end
                end

                S_PART: begin
                    // 粒子浮现：与散灭同阈值同顺序，全部浮现后回 IDLE
                    if (req_edge) begin
                        O_fade_ack <= ~O_fade_ack;
                        state      <= S_IDLE;
                        mix        <= 8'd255;
                    end
                    else if (frame_edge) begin
                        if (part_f >= 4'd15) begin
                            state <= S_IDLE;            // 全部块浮现
                        end
                        else begin
                            part_f <= part_f + 4'd1;
                        end
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

endmodule
