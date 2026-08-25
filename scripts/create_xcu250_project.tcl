# ============================================================================
# Prolepsis - Create a Vivado SystemVerilog project for the XCU250
# ============================================================================

set project_name "Prolepsis"
set repo_root    [file normalize [file join [file dirname [info script]] ..]]
set project_dir  [file join $repo_root "build" $project_name]
set part         "xcu250-figd2104-2L-e"

file mkdir [file join $repo_root "build"]
create_project $project_name $project_dir -part $part -force

set_property target_language SystemVerilog [current_project]
set_property default_lib xil_defaultlib [current_project]
set_property simulator_language Mixed [current_project]

# Package and interface sources are added first so compile order is explicit.
set rtl_dirs [list \
    "rtl/pkg" \
    "rtl/interfaces" \
    "rtl" \
    "rtl/accelerator" \
    "rtl/bus" \
    "rtl/cache" \
    "rtl/core" \
    "rtl/peripherals" \
    "rtl/thermal" \
]
foreach rtl_dir $rtl_dirs {
    set sv_files [glob -nocomplain [file join $repo_root $rtl_dir "*.sv"]]
    if {[llength $sv_files] > 0} {
        add_files -norecurse $sv_files
    }
}

set_property include_dirs [list \
    [file join $repo_root "rtl"] \
    [file join $repo_root "rtl/pkg"] \
    [file join $repo_root "rtl/bus"] \
] [get_filesets sources_1]

set boot_hex [file join $repo_root "sw" "boot.hex"]
if {[file exists $boot_hex]} {
    add_files -norecurse $boot_hex
}

set xdc_file [file join $repo_root "constraints" "xcu250.xdc"]
if {[file exists $xdc_file]} {
    add_files -fileset constrs_1 $xdc_file
}

set_property top orionrv [current_fileset]
update_compile_order -fileset sources_1
save_project_as -force $project_name $project_dir

puts "Created $project_name at $project_dir"
puts "Target part: $part"
puts "Top module: orionrv"
