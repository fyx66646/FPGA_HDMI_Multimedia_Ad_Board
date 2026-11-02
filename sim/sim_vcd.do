# 导出仿真波形 VCD(修复后), 用于绘制报告用仿真图
vlib work
vmap work work
vlog -work work spectrum_core_sim.v tb_spectrum_core.v
vsim -voptargs=+acc -c work.tb_spectrum_core
vcd file sim_wave.vcd
vcd add /tb_spectrum_core/clk
vcd add /tb_spectrum_core/rst
vcd add /tb_spectrum_core/sample_valid
vcd add /tb_spectrum_core/sample_left
vcd add /tb_spectrum_core/levels
vcd add /tb_spectrum_core/uut/fstate
vcd add /tb_spectrum_core/uut/fcnt
vcd add /tb_spectrum_core/uut/samp_tgl
vcd add /tb_spectrum_core/uut/waddr
run -all
vcd close
quit -f
