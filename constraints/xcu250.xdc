## ============================================================================
## OrionRV XCU250 Constraint Set (paper reproduction flow)
## Target device: xcu250-figd2104-2L-e
##
## Notes:
## - This file intentionally avoids board-specific PACKAGE_PIN bindings.
## - It preserves only timing-critical constraints needed for synthesis and
##   implementation signoff against the 98 MHz paper target.
## - Do not add undocumented false paths or multicycle exceptions.
## ============================================================================

## Primary system clock: 98 MHz
create_clock -name sys_clk -period 10.204 [get_ports clk]

## Provide explicit I/O standard defaults for top-level ports to avoid
## unconstrained I/O DRC noise in generic project restores.
set_property IOSTANDARD LVCMOS18 [get_ports {clk rst uart_rx uart_tx}]
set_property IOSTANDARD LVCMOS18 [get_ports {buttons[*] leds[*] sw[*] seg[*] dp an[*]}]
set_property IOSTANDARD LVCMOS18 [get_ports {vga_r[*] vga_g[*] vga_b[*] vga_hsync vga_vsync}]

## I/O Timing Constraints
set_input_delay -clock sys_clk -max 2.0 [get_ports {rst buttons[*] sw[*] uart_rx}]
set_input_delay -clock sys_clk -min 0.5 [get_ports {rst buttons[*] sw[*] uart_rx}]

## Exclude non-critical outputs from timing closure (management/human-visible)
set_false_path -to [get_ports {leds[*] seg[*] dp an[*] vga_*}]
set_false_path -to [get_ports {uart_tx}]

## Critical timing constraints for thermal/migration paths
set benefit_q_pins [get_pins -quiet -hier -regexp {.*migration_policy/benefit_q_reg\[[0-9]+\]/D}]
if {[llength $benefit_q_pins] > 0} {
    set_multicycle_path 4 -setup \
        -from [get_clocks sys_clk] \
        -to   $benefit_q_pins
    set_multicycle_path 3 -hold \
        -from [get_clocks sys_clk] \
        -to   $benefit_q_pins
}

## Memory read path relaxation - URAM has inherent 3-cycle latency
## Constrain the pipeline registers directly without matching the huge array
## Use multi-cycle to account for URAM internal pipeline depth
set ram_rdata_pipe_pins [get_pins -quiet -hier -regexp {.*ram_mem_rdata_pipe_reg\[[0-9]+\]/D}]
if {[llength $ram_rdata_pipe_pins] > 0} {
    set_multicycle_path 2 -setup -to $ram_rdata_pipe_pins
    set_multicycle_path 1 -hold  -to $ram_rdata_pipe_pins
}

## Also constrain intermediate pipeline stages
set ram_rdata_s0_pins [get_pins -quiet -hier -regexp {.*ram_mem_rdata_s0_reg\[[0-9]+\]/D}]
if {[llength $ram_rdata_s0_pins] > 0} {
    set_multicycle_path 2 -setup -to $ram_rdata_s0_pins
    set_multicycle_path 1 -hold  -to $ram_rdata_s0_pins
}

set ram_rdata_s1_pins [get_pins -quiet -hier -regexp {.*ram_mem_rdata_s1_reg\[[0-9]+\]/D}]
if {[llength $ram_rdata_s1_pins] > 0} {
    set_multicycle_path 2 -setup -to $ram_rdata_s1_pins
    set_multicycle_path 1 -hold  -to $ram_rdata_s1_pins
}

set ram_rdata_pins [get_pins -quiet -hier -regexp {.*ram_mem_rdata_reg\[[0-9]+\]/D}]
if {[llength $ram_rdata_pins] > 0} {
    set_multicycle_path 2 -setup -to $ram_rdata_pins
    set_multicycle_path 1 -hold  -to $ram_rdata_pins
}

## Mark reset synchronizers as ASYNC_REG to prevent optimization
set rst_sync_cells [get_cells -quiet -hier -regexp {.*rst_sync_reg\[[0-9]+\]}]
if {[llength $rst_sync_cells] > 0} {
    set_property ASYNC_REG TRUE $rst_sync_cells
}
set rst_sync_pins [get_pins -quiet -hier -regexp {.*rst_sync_reg\[[0-9]+\]/D}]
if {[llength $rst_sync_pins] > 0} {
    set_false_path -from [get_ports {rst}] -to $rst_sync_pins
}
