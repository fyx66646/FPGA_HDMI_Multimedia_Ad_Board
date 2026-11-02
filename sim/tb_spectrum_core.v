// ============================================================================
// tb_spectrum_core : spectrum_core(64 点 FFT 音频频谱)的 ModelSim 仿真
// 激励: 1875Hz 正弦(恰好落在第 5 频点, 375Hz x 5), 样本按真实链路
// "每样本连发两帧"的方式输入, 每帧 64 个有效样本。
// 检查: 一帧 FFT 完成后打印全部频点电平并给出结论。
//   注意: 仿真已验证 RTL 行为与理论 DFT 存在偏差(能量未集中在第 5 频点),
//   详见 sim/README.md 的"已知发现"章节; 本 testbench 的作用是忠实复现
//   并观察该行为, FAIL 提示即"发现偏差", 不代表仿真环境故障。
// 运行: vsim -c -do sim_batch.do   (批处理)
//       vsim -do sim_gui.do        (带波形窗口)
// ============================================================================
`timescale 1ns / 1ps

module tb_spectrum_core;

reg              clk = 0;
reg              rst = 1;
reg              sample_valid = 0;
reg signed [23:0] sample_left = 0;
wire [191:0]     levels;

spectrum_core uut (
    .I_clk          (clk),          // video_clk 25MHz
    .I_rst          (rst),
    .I_sample_valid (sample_valid),
    .I_sample_left  (sample_left),
    .O_levels       (levels)
);

localparam CLK_PERIOD = 40;         // 25MHz = 40ns

always #(CLK_PERIOD / 2) clk = ~clk;

// ---------------- 激励: 1875Hz 正弦, 振幅 0.4 满幅 ----------------
reg signed [15:0] sine_q [0:63];
integer i;
real    pi = 3.141592653589793;

initial begin
    for (i = 0; i < 64; i = i + 1)
        sine_q[i] = $rtoi(0.4 * 32767.0 * $sin(2.0 * pi * 5.0 * i / 64.0));
end

initial begin
    // 复位
    repeat (8) @(negedge clk);
    rst = 0;
    repeat (4) @(negedge clk);

    // 64 个样本, 每个连发两拍有效(样本重复法, 模块每对只取第一个)
    for (i = 0; i < 64; i = i + 1) begin
        sample_left = $signed(sine_q[i]) <<< 8;   // PCM 24bit, 模块取 [23:8]
        @(negedge clk);
        sample_valid = 1'b1;
        repeat (2) @(posedge clk);                // 覆盖两个采样沿
        @(negedge clk);                           // 在 negedge 清零, 避开采样沿竞态
        sample_valid = 1'b0;
    end
    sample_left = 0;

    // 等待一帧 FFT 完成(约 230 拍)并留出余量
    repeat (2000) @(negedge clk);
    check_result;
    $finish;
end

// ---------------- 自检: 打印频点电平并给出结论 ----------------
task check_result;
    integer b;
    reg [5:0]  lv;
    reg [4:0]  peak;
    reg [5:0]  pv;
    begin
        peak = 0;
        pv   = 0;
        $display("==============================");
        $display(" 频点电平表 (bin : level)");
        for (b = 0; b < 32; b = b + 1) begin
            lv = levels[b * 6 +: 6];
            if (lv != 0)
                $display("   bin %0d = %0d", b, lv);
            if (lv > pv) begin
                pv   = lv;
                peak = b[4:0];
            end
        end
        $display("------------------------------");
        $display(" 峰值频点 bin %0d, 电平 %0d", peak, pv);
        if ((peak == 5'd5) && (pv > 6'd20)) begin
            $display(" === PASS ===  能量集中在 bin 5(1875Hz), 与理论一致");
        end
        else begin
            $display(" === FAIL ===  能量未集中在 bin 5: 与理论 DFT 有偏差");
            $display("             (已知发现, 详见 sim/README.md, 不代表仿真环境故障)");
        end
        $display("==============================");
    end
endtask

endmodule
