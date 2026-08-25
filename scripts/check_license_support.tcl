# -----------------------------------------------------------------------------
# Vivado license/device availability check for Prolepsis reconstruction flow.
# Run from Vivado Tcl console:
#   source scripts/check_license_support.tcl
# -----------------------------------------------------------------------------

set target_part "xcu250-figd2104-2L-e"
puts "Vivado version: [version -short]"
puts "Checking support for part: $target_part"

set tmp_proj_dir [file normalize "./build/vivado/_license_probe_xcu250"]
if {[file exists $tmp_proj_dir]} {
    file delete -force $tmp_proj_dir
}

set ok 1
if {[catch {create_project _license_probe_xcu250 $tmp_proj_dir -part $target_part} err]} {
    puts "ERROR: Unable to create project for $target_part"
    puts "DETAIL: $err"
    set ok 0
}

if {$ok} {
    puts "PASS: Vivado can target $target_part in this installation."
    close_project
}

if {[file exists $tmp_proj_dir]} {
    file delete -force $tmp_proj_dir
}
