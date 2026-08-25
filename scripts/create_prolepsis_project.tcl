# Backward-compatible entry point.  The XCU250 flow is maintained in one
# script so both documented commands create the same SystemVerilog project.
set script_dir [file dirname [file normalize [info script]]]
source [file join $script_dir "create_xcu250_project.tcl"]
