# ModelSim 图形界面仿真(带波形窗口)
# 用法: vsim -do sim_gui.do
vlib work
vmap work work
vlog -work work spectrum_core_sim.v tb_spectrum_core.v
vsim -voptargs=+acc work.tb_spectrum_core
add wave -position end sim:/tb_spectrum_core/clk
add wave -position end sim:/tb_spectrum_core/rst
add wave -position end sim:/tb_spectrum_core/sample_valid
add wave -position end -radix decimal sim:/tb_spectrum_core/sample_left
add wave -position end -radix hex sim:/tb_spectrum_core/levels
add wave -position end -radix unsigned sim:/tb_spectrum_core/uut/fstate
add wave -position end -radix unsigned sim:/tb_spectrum_core/uut/fcnt
add wave -position end -radix unsigned sim:/tb_spectrum_core/uut/waddr
run -all
