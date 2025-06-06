# file: wave.do
# (c) Copyright 2009 - 2011 Xilinx, Inc. All rights reserved.
# 
# This file contains confidential and proprietary information
# of Xilinx, Inc. and is protected under U.S. and
# international copyright and other intellectual property
# laws.
# 
# DISCLAIMER
# This disclaimer is not a license and does not grant any
# rights to the materials distributed herewith. Except as
# otherwise provided in a valid license issued to you by
# Xilinx, and to the maximum extent permitted by applicable
# law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
# WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
# AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
# BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
# INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
# (2) Xilinx shall not be liable (whether in contract or tort,
# including negligence, or under any other theory of
# liability) for any loss or damage of any kind or nature
# related to, arising under or in connection with these
# materials, including for any direct, or any indirect,
# special, incidental, or consequential loss or damage
# (including loss of data, profits, goodwill, or any type of
# loss or damage suffered as a result of any action brought
# by a third party) even if such damage or loss was
# reasonably foreseeable or Xilinx had been advised of the
# possibility of the same.
# 
# CRITICAL APPLICATIONS
# Xilinx products are not designed or intended to be fail-
# safe, or for use in any application requiring fail-safe
# performance, such as life-support or safety devices or
# systems, Class III medical devices, nuclear facilities,
# applications related to the deployment of airbags, or any
# other applications that could lead to death, personal
# injury, or severe property or environmental damage
# (individually and collectively, "Critical
# Applications"). Customer assumes the sole risk and
# liability of any use of Xilinx products in Critical
# Applications, subject only to applicable laws and
# regulations governing limitations on product liability.
# 
# THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
# PART OF THIS FILE AT ALL TIMES.


onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -format Literal -radix unsigned /selectio_if_wiz_v4_1_tb/dut/sys_w
add wave -noupdate -format Literal -radix unsigned /selectio_if_wiz_v4_1_tb/dut/dev_w
add wave -noupdate -format Logic   /selectio_if_wiz_v4_1_tb/IO_RESET
add wave -noupdate -format Logic   /selectio_if_wiz_v4_1_tb/CLK_RESET
add wave -noupdate -format Logic   /selectio_if_wiz_v4_1_tb/CLK_IN
add wave -noupdate -format Logic   /selectio_if_wiz_v4_1_tb/CLK_IN_P
add wave -noupdate -format Logic   /selectio_if_wiz_v4_1_tb/CLK_IN_N
add wave -noupdate -format Logic   /selectio_if_wiz_v4_1_tb/dut/CLK_DIV_OUT
add wave -noupdate -format Literal /selectio_if_wiz_v4_1_tb/DATA_OUT_TO_PINS_P
add wave -noupdate -format Literal /selectio_if_wiz_v4_1_tb/DATA_OUT_TO_PINS_N
add wave -noupdate -format Literal /selectio_if_wiz_v4_1_tb/dut/io_inst/oserdes_d
add wave -noupdate -format Literal /selectio_if_wiz_v4_1_tb/dut/DATA_OUT_FROM_DEVICE
add wave -noupdate -format Logic   /selectio_if_wiz_v4_1_tb/dut/IO_RESET
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {370825 ps} 0}
configure wave -namecolwidth 230
configure wave -valuecolwidth 100
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ns
update
WaveRestoreZoom {0 ps} {3169444 ps}
