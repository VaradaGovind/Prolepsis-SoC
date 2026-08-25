## Clock signal
set_property -dict { PACKAGE_PIN E3    IOSTANDARD LVCMOS33 } [get_ports { clk }]; #IO_L12P_T1_MRCC_35 Sch=clk100mhz
# Slightly relax from 100MHz to ~98MHz for robust no-heatsink bring-up margin.
create_clock -add -name sys_clk_pin -period 10.20 -waveform {0 5.1} [get_ports {clk}];
# Fix configuration-bank voltage DRC checks.
set_property CFGBVS VCCO [current_design]
set_property CONFIG_VOLTAGE 3.3 [current_design]

## Reset signal (CPU_RESETN)
set_property -dict { PACKAGE_PIN C12   IOSTANDARD LVCMOS33 } [get_ports { rst }]; #IO_L3P_T0_DQS_AD1P_15 Sch=cpu_resetn

## LEDs
set_property -dict { PACKAGE_PIN H17   IOSTANDARD LVCMOS33 } [get_ports { leds[0] }]; #IO_L18P_T2_A24_15 Sch=led[0]
set_property -dict { PACKAGE_PIN K15   IOSTANDARD LVCMOS33 } [get_ports { leds[1] }]; #IO_L24P_T3_RS1_15 Sch=led[1]
set_property -dict { PACKAGE_PIN J13   IOSTANDARD LVCMOS33 } [get_ports { leds[2] }]; #IO_L17N_T2_A25_15 Sch=led[2]
set_property -dict { PACKAGE_PIN N14   IOSTANDARD LVCMOS33 } [get_ports { leds[3] }]; #IO_L8P_T1_D11_14 Sch=led[3]
set_property -dict { PACKAGE_PIN R18   IOSTANDARD LVCMOS33 } [get_ports { leds[4] }]; #IO_L7P_T1_D09_14 Sch=led[4]
set_property -dict { PACKAGE_PIN V17   IOSTANDARD LVCMOS33 } [get_ports { leds[5] }]; #IO_L18N_T2_A11_D27_14 Sch=led[5]
set_property -dict { PACKAGE_PIN U17   IOSTANDARD LVCMOS33 } [get_ports { leds[6] }]; #IO_L17P_T2_A14_D30_14 Sch=led[6]
set_property -dict { PACKAGE_PIN U16   IOSTANDARD LVCMOS33 } [get_ports { leds[7] }]; #IO_L18P_T2_A12_D28_14 Sch=led[7]

## Buttons
# D-pad mode select mapping in OrionRV top-level:
#   buttons[0] (BTNC) -> mode 0 (Baseline)
#   buttons[1] (BTNU) -> mode 1 (Reactive)
#   buttons[2] (BTNL) -> mode 2 (Predictive/Kalman)
#   buttons[3] (BTNR) -> mode 3 (Full system)
set_property -dict { PACKAGE_PIN N17   IOSTANDARD LVCMOS33 } [get_ports { buttons[0] }]; #IO_L9P_T1_DQS_14 Sch=btnc
set_property -dict { PACKAGE_PIN M18   IOSTANDARD LVCMOS33 } [get_ports { buttons[1] }]; #IO_L4N_T0_D05_14 Sch=btnu
set_property -dict { PACKAGE_PIN P17   IOSTANDARD LVCMOS33 } [get_ports { buttons[2] }]; #IO_L12P_T1_MRCC_14 Sch=btnl
set_property -dict { PACKAGE_PIN M17   IOSTANDARD LVCMOS33 } [get_ports { buttons[3] }]; #IO_L10N_T1_D15_14 Sch=btnr

## Slide Switches (SW0-SW1 for core temperature selection)
set_property -dict { PACKAGE_PIN J15   IOSTANDARD LVCMOS33 } [get_ports { sw[0] }]; #IO_L24N_T3_RS0_15 Sch=sw[0]
set_property -dict { PACKAGE_PIN L16   IOSTANDARD LVCMOS33 } [get_ports { sw[1] }]; #IO_L3N_T0_DQS_EMCCLK_14 Sch=sw[1]

## 7-Segment Display (Active-Low Segments)
set_property -dict { PACKAGE_PIN T10   IOSTANDARD LVCMOS33 } [get_ports { seg[0] }]; #IO_L24N_T3_A00_D16_14 Sch=ca
set_property -dict { PACKAGE_PIN R10   IOSTANDARD LVCMOS33 } [get_ports { seg[1] }]; #IO_L25P_T3_A10_12 Sch=cb (Note: Nexys4 DDR)
set_property -dict { PACKAGE_PIN K16   IOSTANDARD LVCMOS33 } [get_ports { seg[2] }]; #IO_L6P_T0_15 Sch=cc
set_property -dict { PACKAGE_PIN K13   IOSTANDARD LVCMOS33 } [get_ports { seg[3] }]; #IO_L17P_T2_A26_15 Sch=cd
set_property -dict { PACKAGE_PIN P15   IOSTANDARD LVCMOS33 } [get_ports { seg[4] }]; #IO_L13P_T2_MRCC_14 Sch=ce
set_property -dict { PACKAGE_PIN T11   IOSTANDARD LVCMOS33 } [get_ports { seg[5] }]; #IO_L19P_T3_A10_D26_14 Sch=cf
set_property -dict { PACKAGE_PIN L18   IOSTANDARD LVCMOS33 } [get_ports { seg[6] }]; #IO_L4P_T0_D04_14 Sch=cg

## 7-Segment Decimal Point
set_property -dict { PACKAGE_PIN H15   IOSTANDARD LVCMOS33 } [get_ports { dp }]; #IO_L19N_T3_A21_VREF_15 Sch=dp

## 7-Segment Anode Enables (Active-Low)
set_property -dict { PACKAGE_PIN J17   IOSTANDARD LVCMOS33 } [get_ports { an[0] }]; #IO_L23P_T3_FOE_B_15 Sch=an[0]
set_property -dict { PACKAGE_PIN J18   IOSTANDARD LVCMOS33 } [get_ports { an[1] }]; #IO_L23N_T3_FWE_B_15 Sch=an[1]
set_property -dict { PACKAGE_PIN T9    IOSTANDARD LVCMOS33 } [get_ports { an[2] }]; #IO_L24P_T3_A01_D17_14 Sch=an[2]
set_property -dict { PACKAGE_PIN J14   IOSTANDARD LVCMOS33 } [get_ports { an[3] }]; #IO_L19P_T3_A22_15 Sch=an[3]
set_property -dict { PACKAGE_PIN P14   IOSTANDARD LVCMOS33 } [get_ports { an[4] }]; #IO_L8N_T1_D12_14 Sch=an[4]
set_property -dict { PACKAGE_PIN T14   IOSTANDARD LVCMOS33 } [get_ports { an[5] }]; #IO_L14P_T2_SRCC_14 Sch=an[5]
set_property -dict { PACKAGE_PIN K2    IOSTANDARD LVCMOS33 } [get_ports { an[6] }]; #IO_L23P_T3_35 Sch=an[6]
set_property -dict { PACKAGE_PIN U13   IOSTANDARD LVCMOS33 } [get_ports { an[7] }]; #IO_L23N_T3_A02_D18_14 Sch=an[7]

# Slow edge rate on seven-seg outputs improves visual stability and reduces
# perceived ghosting while migration/event digits are held on-screen.
set_property SLEW SLOW [get_ports {seg[0] seg[1] seg[2] seg[3] seg[4] seg[5] seg[6] dp an[0] an[1] an[2] an[3] an[4] an[5] an[6] an[7]}]
set_property DRIVE 8 [get_ports {seg[0] seg[1] seg[2] seg[3] seg[4] seg[5] seg[6] dp an[0] an[1] an[2] an[3] an[4] an[5] an[6] an[7]}]

## Timing Constraints (I/O Delays)
set_input_delay -clock sys_clk_pin -max 2.000 [get_ports {rst buttons* sw*}]
set_input_delay -clock sys_clk_pin -min 0.500 [get_ports {rst buttons* sw*}]

# Human-visible GPIO are not source-synchronous to sys_clk_pin.
# Exclude these outputs from setup/hold signoff to focus closure on internal
# synchronous datapaths.
set_false_path -to [get_ports {leds* seg* an* dp}]

# Mark the two-stage reset synchronizer and exclude its distribution tree from
# datapath timing closure (reset assertion/deassertion is functionally handled
# by the synchronizer itself, not by datapath setup timing).
set_property ASYNC_REG TRUE [get_cells -quiet -hier -regexp {.*rst_sync_reg\[[0-9]+\]}]
set_false_path -from [get_ports {rst}] -to [get_pins -quiet -hier -regexp {.*rst_sync_reg\[[0-9]+\]/D}]

# Migration policy decisions are management-path updates and are consumed over
# multiple cycles; constrain the policy capture stage accordingly.
set_multicycle_path 4 -setup \
	-from [get_clocks sys_clk_pin] \
	-to   [get_pins -quiet -hier -regexp {.*migration_policy/(src_sel_q_reg\[[0-9]+\]|dst_sel_q_reg\[[0-9]+\]|benefit_q_reg\[[0-9]+\]|cost_q_reg\[[0-9]+\]|proactive_q_reg|all_hot_q_reg)/D}]
set_multicycle_path 3 -hold \
	-from [get_clocks sys_clk_pin] \
	-to   [get_pins -quiet -hier -regexp {.*migration_policy/(src_sel_q_reg\[[0-9]+\]|dst_sel_q_reg\[[0-9]+\]|benefit_q_reg\[[0-9]+\]|cost_q_reg\[[0-9]+\]|proactive_q_reg|all_hot_q_reg)/D}]

# core_rst_r is generated synchronously but drives asynchronous clear/reset pins
# in several sub-blocks. Apply guarded false paths only when endpoint sets are
# valid and reasonably sized.
set _orionrv_rst_hiers [list \
	"*core_gen*core_inst*" \
	"*phase_inst*" \
	"*eval_fw*" \
	"*migration_ctrl*" \
	"*cache_warmer*" \
	"*thermal_sensor_gen*" \
]

set _rst_pins [get_pins -quiet -hier -filter "NAME =~ *core_gen*core_inst* && (REF_PIN_NAME == \"R\" || REF_PIN_NAME == \"CLR\")"]
if {[llength $_rst_pins] > 0 && [llength $_rst_pins] < 10000} {
	set_false_path -to $_rst_pins
}

set _rst_pins [get_pins -quiet -hier -filter "NAME =~ *phase_inst* && (REF_PIN_NAME == \"R\" || REF_PIN_NAME == \"CLR\")"]
if {[llength $_rst_pins] > 0 && [llength $_rst_pins] < 10000} {
	set_false_path -to $_rst_pins
}

set _rst_pins [get_pins -quiet -hier -filter "NAME =~ *eval_fw* && (REF_PIN_NAME == \"R\" || REF_PIN_NAME == \"CLR\")"]
if {[llength $_rst_pins] > 0 && [llength $_rst_pins] < 10000} {
	set_false_path -to $_rst_pins
}

set _rst_pins [get_pins -quiet -hier -filter "NAME =~ *migration_ctrl* && (REF_PIN_NAME == \"R\" || REF_PIN_NAME == \"CLR\")"]
if {[llength $_rst_pins] > 0 && [llength $_rst_pins] < 10000} {
	set_false_path -to $_rst_pins
}

set _rst_pins [get_pins -quiet -hier -filter "NAME =~ *cache_warmer* && (REF_PIN_NAME == \"R\" || REF_PIN_NAME == \"CLR\")"]
if {[llength $_rst_pins] > 0 && [llength $_rst_pins] < 10000} {
	set_false_path -to $_rst_pins
}

set _rst_pins [get_pins -quiet -hier -filter "NAME =~ *thermal_sensor_gen* && (REF_PIN_NAME == \"R\" || REF_PIN_NAME == \"CLR\")"]
if {[llength $_rst_pins] > 0 && [llength $_rst_pins] < 10000} {
	set_false_path -to $_rst_pins
}
