# ==========================================================
# OrionRV Boot Migration + Stress Program
# ==========================================================
# Goal:
#  - Exercise migration/eval MMIO flow continuously on FPGA
#  - Keep ALU/RV32M/branch pressure active between migration commands
#
# MMIO map used (base = 0x9000_0000):
#   0x00 : LED mirror/debug writeback
#   0x50 : Migration command (start/clear + src/dst fields)
#   0x54 : Migration status (busy/done/error)
#   0x58 : Last migration cycles
#   0x5C : Migration count
#   0x68 : Cache warm cycles
#   0x70 : Eval mode control
#   0xC0 : Eval run-done pulse
# ==========================================================

.section .text
.globl _start

_start:
    # Trap vector setup
    la     t0, trap_handler
    csrw   mtvec, t0

    # Base pointers
    lui    s0, 0x20000         # s0 = 0x2000_0000 (RAM base, reserved)
    lui    s1, 0x90000         # s1 = 0x9000_0000 (MMIO base)

    # Enable full-stack eval mode.
    addi   t0, x0, 3
    sw     t0, 0x70(s1)

    # Migration route state (src -> dst), round-robin over cores 0..4.
    addi   s2, x0, 0           # src
    addi   s3, x0, 1           # dst

    # Stress seeds
    lui    s4, 0x12345
    addi   s4, s4, 0x678
    lui    s5, 0x89ABD
    addi   s5, s5, -529
    addi   s6, x0, 0           # loop counter

main_loop:
    # Build migration command: bit0=start, src[10:8], dst[14:12].
    slli   t0, s2, 8
    slli   t1, s3, 12
    or     t0, t0, t1
    addi   t0, t0, 1
    sw     t0, 0x50(s1)

wait_busy_high:
    lw     t2, 0x54(s1)
    andi   t3, t2, 8
    bne    t3, x0, wait_busy_low
    jal    x0, wait_busy_high

wait_busy_low:
    lw     t2, 0x54(s1)
    andi   t3, t2, 8
    bne    t3, x0, wait_busy_low

    # Clear sticky halt so migrated source can run again in later rounds.
    addi   t0, x0, 2
    sw     t0, 0x50(s1)

    # Mix migration telemetry and reflect on LEDs for board visibility.
    lw     a0, 0x58(s1)
    lw     a1, 0x5C(s1)
    lw     a6, 0x68(s1)
    xor    a2, a0, a1
    xor    a2, a2, a6
    sw     a2, 0x00(s1)

    # ALU + RV32M + branch stress between migration commands.
    add    t0, s4, s5
    xor    t1, s4, s5
    mul    a3, s4, s5
    mulh   a4, s4, s5
    srl    a5, a3, s3
    add    s4, t0, a4
    xor    s5, t1, a5
    andi   t4, s4, 1
    bne    t4, x0, mix_odd
    add    s5, s5, s4
    jal    x0, mix_join

mix_odd:
    xor    s5, s5, s4

mix_join:
    # Next route: src=(src+1)%5, dst=(src+1)%5.
    addi   s2, s2, 1
    addi   t5, x0, 5
    blt    s2, t5, src_ok
    addi   s2, x0, 0

src_ok:
    addi   s3, s2, 1
    blt    s3, t5, dst_ok
    addi   s3, x0, 0

dst_ok:
    addi   s6, s6, 1
    andi   t6, s6, 15
    bne    t6, x0, main_loop

    # Periodic eval snapshot pulse.
    addi   t0, x0, 1
    sw     t0, 0xC0(s1)
    jal    x0, main_loop

    nop
    nop
    nop

.balign 16
trap_handler:
    csrr   t0, mcause
    lui    t1, 0x80000
    addi   t1, t1, 0x10
    bne    t0, t1, trap_exit

    # Brief cooldown before returning from trap.
    lui    t3, 0x10
cool_down:
    addi   t3, t3, -1
    bne    t3, x0, cool_down

trap_exit:
    mret
