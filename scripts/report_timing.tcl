# ============================================================================
# Prolepsis - Post-implementation timing report bundle
#
# Source this script after opening an implemented design in Vivado:
#   source scripts/report_timing.tcl
# ============================================================================

set script_dir [file dirname [file normalize [info script]]]
set repo_root  [file normalize [file join $script_dir ..]]
set report_dir [file join $repo_root "reports"]
file mkdir $report_dir

report_timing_summary -delay_type min_max -max_paths 20 -nworst 1 \
    -report_unconstrained -check_timing_verbose \
    -file [file join $report_dir "timing_summary.rpt"]
check_timing -verbose -file [file join $report_dir "check_timing.rpt"]
report_methodology -file [file join $report_dir "methodology_report.rpt"]
report_multicycle_path -setup -file [file join $report_dir "multicycle_setup.rpt"]
report_multicycle_path -hold  -file [file join $report_dir "multicycle_hold.rpt"]

puts "Timing reports written to $report_dir"
