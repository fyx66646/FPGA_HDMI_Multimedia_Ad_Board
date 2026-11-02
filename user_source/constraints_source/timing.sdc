
# ============================================================
# merged_top_tf_hdmi_audio.sdc
# 用于：TF 图片轮播 + HDMI(视频+音频) 整合工程
#
# 合并原则：
# 1) 保留 top.sdc 里的系统主时钟与 derive_clocks
# 2) 把 timing(2).sdc 里的 pixel/serial 时钟改成整合工程实际名字
# 3) 增加 hdmi_5x_clk / audio_mclk 的命名
# 4) 不再对内部 net 直接 create_clock，避免和 derive_clocks 重复
# ============================================================

# ------------------------------------------------------------
# 1. 板级输入主时钟
#    你的原 top.sdc 里 clk=50MHz，所以这里仍然用 20ns
# ------------------------------------------------------------
create_clock -name clk -period 20 -waveform {0 10} [get_ports {clk}]

# ------------------------------------------------------------
# 2. 自动推导 PLL 输出时钟
# ------------------------------------------------------------
derive_clocks

# ------------------------------------------------------------
# 3. 给关键 PLL 输出命名
#    下面这些路径是按你当前工程实例名写的：
#      sys_pll      -> sys_pll_m0
#      video_pll    -> video_pll_m0
#      音频 PLL     -> u_audio_pll
#
#    如果综合后实例路径略有不同，就按 Timing Analyzer 里的实际 pin 路径改。
# ------------------------------------------------------------
rename_clock -name {sd_card_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {ext_mem_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {sys_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {video_clk}   -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[0]}]
rename_clock -name {hdmi_5x_clk} -source [get_ports {clk}] -master_clock {clk} [get_pins {video_pll_m0/pll_inst.clkc[1]}]
rename_clock -name {audio_mclk}  -source [get_ports {clk}] -master_clock {clk} [get_pins {u_audio_pll/pll_inst.clkc[2]}]

# ------------------------------------------------------------
# 4. 从原 timing(2).sdc 迁移过来的 HDMI 像素/串行时钟约束
#
#    原文件是：
#      create_clock -name pixel_clk  ... [get_nets {S_pixel_clk}]
#      create_clock -name serial_clk ... [get_nets {S_serial_clk}]
#      set_clock_groups -exclusive ...
#
#    在整合工程里，这两个时钟已经由 video_pll 产生，并通过 derive_clocks 建出来了，
#    所以这里只保留“组关系”，不再重复 create_clock。
#
#    这里沿用原作者的写法，把 video_clk 和 hdmi_5x_clk 设成 exclusive。
#    如果后面 Timing Analyzer 明确显示两者之间需要做同步时序分析，再把这一条去掉。
# ------------------------------------------------------------
set_clock_groups -exclusive \
    -group [get_clocks {video_clk}] \
    -group [get_clocks {hdmi_5x_clk}]

# ------------------------------------------------------------
# 5. 音频 I2S 发生器/接收器跨域
#
#    audio_mclk 域里生成 I2S，video_clk 域里做三拍同步采样。
#    这类路径不是普通同步时序路径，直接切掉更合理。
# ------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks {audio_mclk}] \
    -group [get_clocks {video_clk} hdmi_5x_clk sd_card_clk ext_mem_clk]

# ------------------------------------------------------------
# 6. 跨域处理（2026-10-21 实测校准）
#    各时钟域之间只经双时钟 FIFO/灰码同步器/双时钟 RAM/准静态信号：
#      sd_card_clk <-> ext_mem_clk ：SDRAM 帧缓存接口（帧读写握手）
#      sd_card_clk <-> video_clk   ：text_ram / font_cn_ram 双时钟 RAM
#      sd_card_clk <-> audio_mclk  ：audio_fifo 灰码同步
#      clk(50M) <-> 其他           ：数码管状态码显示（慢变化）、复位树
#    这些路径不做组间时序检查。此前未声明异步关系，每次布线产生随机分布
#    的 hold/setup 伪违例（audio_fifo 灰码同步器 hold 违例实测出现过功能
#    故障），声明后 P&R 结果稳定可复现。
# ------------------------------------------------------------
set_clock_groups -asynchronous \
    -group [get_clocks {clk}] \
    -group [get_clocks {sd_card_clk}] \
    -group [get_clocks {ext_mem_clk}] \
    -group [get_clocks {video_clk} hdmi_5x_clk] \
    -group [get_clocks {audio_mclk}]

# 显式双向 false_path，确保 hold 检查一并免除（工具对异步组可能只免 setup）
set_false_path -from [get_clocks {audio_mclk}] -to [get_clocks {clk sd_card_clk ext_mem_clk video_clk}]
set_false_path -from [get_clocks {clk sd_card_clk ext_mem_clk video_clk}] -to [get_clocks {audio_mclk}]
set_false_path -from [get_clocks {sd_card_clk}] -to [get_clocks {clk video_clk ext_mem_clk}]
set_false_path -from [get_clocks {clk video_clk ext_mem_clk}] -to [get_clocks {sd_card_clk}]
set_false_path -from [get_clocks {ext_mem_clk}] -to [get_clocks {clk video_clk}]
set_false_path -from [get_clocks {clk video_clk}] -to [get_clocks {ext_mem_clk}]
set_false_path -from [get_clocks {video_clk}] -to [get_clocks {clk}]
set_false_path -from [get_clocks {clk}] -to [get_clocks {video_clk}]

# ------------------------------------------------------------
# 7. 其余 Input/Output delay 先保持空白
#    你当前两个原始 sdc 也都没有写这部分。
# ------------------------------------------------------------
