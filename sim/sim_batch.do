# ModelSim 批处理仿真(无图形界面)
# 用法: vsim -c -do sim_batch.do
vlib work
vmap work work
vlog -work work spectrum_core_sim.v tb_spectrum_core.v
vsim -c work.tb_spectrum_core
run -all
quit -f
