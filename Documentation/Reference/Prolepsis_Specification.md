# Prolepsis SoC - Technical Specification

**Version:** 2.0  
**Date:** April 17, 2026

This document summarizes the current Prolepsis implementation status and architecture for FPGA deployment and thermal-control research.

## 1. Overview

Prolepsis is a 4+1 heterogeneous multicore SoC based on RV32IMAC, implemented in Verilog for Xilinx Artix-7 class FPGA targets.

- **Architecture:** 4 P-cores + 1 thermal management E-core
- **ISA:** RV32IMAC (Integer + Multiply/Divide + Atomics + Compressed)
- **Pipeline:** 5-stage in-order with branch prediction
- **Interconnect:** AXI4-Lite with QoS arbitration
- **Cache:** Per-core private I-cache (4KB, next-line prefetch) + per-core private L1D (write-through, store buffer, MSHR, victim buffer, stride prefetcher)
- **Branch Prediction:** GShare BHT + BTB + Return Address Stack
- **Active build RTL size (`files.f`):** 8,659 lines across 36 RTL files
- **Repository RTL size (Phase 7.5 locked scope):** 8,659 lines across 36 RTL files

## 2. Core Architecture (`RV32_Core`)

Each core instance implements a 5-stage pipeline with Phase 7.5 microarchitectural enhancements:

### 2.1 Pipeline Stages

- **IF:** Instruction fetch with PC selection (sequential/branch/jump/prediction), loop buffer override, RV32C decompression
- **ID:** Decode, register reads, immediate generation, JAL resolution
- **EX:** ALU operations, branch resolution, misprediction detection
- **MEM:** Load/store memory access through data cache and AXI path
- **WB:** Register write-back mux and commit path

### 2.2 ISA Support

- **RV32I:** Full 40-instruction base integer ISA
- **RV32M:** 8-instruction multiply/divide extension (pipelined ALU with DSP inference)
- **RV32A:** LR/SC reservation + all 9 AMO instructions (AMOSWAP, AMOADD, AMOAND, AMOOR, AMOXOR, AMOMIN, AMOMAX, AMOMINU, AMOMAXU)
- **RV32C:** Full compressed instruction set (16-bit → 32-bit expansion via `RVC_Decompressor.sv`)

### 2.3 Hazard Handling

- 3-level data forwarding (EX→EX, MEM→EX, WB→EX)
- Load-use stall detection and pipeline bubble insertion
- Branch dependency stall (holds branch in ID until operands are available)
- Misprediction-based control flush (correctly predicted branches cause zero pipeline bubbles)

### 2.4 Branch Prediction (`Branch_Predictor.sv`)

- **GShare BHT:** 64-entry, 2-bit saturating counters indexed by `PC[7:2] XOR GHR[5:0]`
- **BTB:** 64-entry Branch Target Buffer with tag matching — supplies predicted target address
- **RAS:** 8-entry Return Address Stack — pushes on call (`JAL`/`JALR` with `rd=x1/x5`), pops on return (`JALR` with `rs1=x1/x5`, `rd=x0`)
- **GHR:** 6-bit Global History Register, shifts in actual branch outcomes from EX stage
- **Prediction priority:** RAS return > BHT+BTB > sequential
- **Misprediction detection:** Direction wrong OR target wrong → flush + redirect PC

### 2.5 Hardware Loop Buffer (`Loop_Buffer.sv`)

- 16-instruction circular buffer for capturing tight backward-branch loops
- 3-state FSM: IDLE → FILL → ACTIVE
- When active: serves instructions directly from buffer, I-cache can be clock-gated
- Exports `loop_active` signal for thermal management integration (power savings)
- Invalidated on pipeline flush or loop exit (branch not taken)

### 2.6 RV32C Decompressor (`RVC_Decompressor.sv`)

- Fully combinational 16→32 bit instruction expansion
- Supports all three RV32C quadrants (C0, C1, C2)
- PC increments by +2 for compressed instructions, +4 for normal instructions
- Handles: C.ADDI4SPN, C.LW, C.SW, C.ADDI, C.JAL, C.LI, C.LUI, C.ADDI16SP, C.SRLI, C.SRAI, C.ANDI, C.SUB, C.XOR, C.OR, C.AND, C.J, C.BEQZ, C.BNEZ, C.SLLI, C.LWSP, C.JR, C.MV, C.JALR, C.ADD, C.EBREAK, C.SWSP

### 2.7 CSR Unit

- Machine-mode CSR support: `mstatus`, `mie`, `mtvec`, `mepc`, `mcause`, `mtval`, `mip`, `mcycle`, `minstret`, `mhartid`
- Custom thermal CSRs: `mtherm` (0x7C0), `mtherm_cfg` (0x7C1)
- Hardware Performance Counters (0x7C2–0x7CB): active cycles, throttled cycles, I-cache hit/miss, branches, branch misses, memory stalls, migration events, thermal events, power proxy
- ECALL trap entry and MRET return

### 2.8 Interrupt Handling

- Timer interrupt, external interrupt, and dedicated thermal interrupt paths
- Thermal interrupt: highest priority, non-maskable (`mcause` code `0x8000_0010`)
- Interrupt gating during atomic sequences (between LR and SC)

### 2.9 Migration Support

- Migration-facing register-file access port (`mig_we`, `mig_addr`, `mig_wdata`, `mig_rdata`)
- PC save/restore interface (`pc_save`, `pc_load_en`, `pc_load`)
- Pipeline flush on PC restore for safe task migration

## 3. Memory Subsystem

### 3.1 Instruction Cache (`Instruction_Cache.sv`)

- 4KB direct-mapped, 256 lines × 4 words (16-byte line fills)
- Single-cycle hit latency
- **Next-line prefetch:** Detects access to last word of a cache line → prefetches next sequential line if not already valid
- Prefetch is non-blocking and abortable: demand misses immediately preempt in-flight prefetches
- FSM states: IDLE, FILL, FILL_WAIT, COMPLETE, PF_FILL, PF_FILL_WAIT, PF_COMPLETE

### 3.2 Data Cache (`Data_Cache.sv`)

- Direct-mapped write-through, write-allocate with snoop-invalidate coherence
- 256 lines × 4 words (16-byte lines)
- **1-entry store buffer:** Stores retire immediately to pipeline; write-through drains in background. Store-to-load forwarding on address match
- **MSHR (Miss Status Holding Register):** Tracks one outstanding miss for hit-under-miss capability
- **Victim buffer interface:** Eviction push, miss lookup, and swap-in ports connected to external `Victim_Buffer.sv`
- **Stride prefetch port:** Accepts low-priority prefetch hints from `Stride_Prefetcher.sv`, abortable on demand
- **Store-buffer-aware snoop:** Coherence invalidation also clears matching store buffer entries
- FSM states: IDLE, FILL_REQ/WAIT/COMPLETE, STORE_REQ/WAIT, VICTIM_SWAP, PF_FILL_REQ/WAIT/COMPLETE

### 3.3 Victim Buffer (`Victim_Buffer.sv`)

- 2-entry fully-associative victim buffer for L1D conflict miss reduction
- On L1D eviction: evicted line is pushed to victim buffer (FIFO replacement)
- On L1D miss: victim buffer checked before going to memory
- On victim hit: swap — victim entry moves to L1D, L1D evictee goes to victim buffer (zero memory access)
- Coherence: snoop invalidation also checks and clears victim buffer entries

### 3.4 Stride Prefetcher (`Stride_Prefetcher.sv`)

- 4-entry stride table indexed by PC hash
- Each entry: last_addr, stride, 2-bit confidence counter, valid flag, PC tag
- Stride detection: computes `delta = miss_addr - last_addr`, increments confidence on match
- Prefetch trigger: only after 3 consecutive stride confirmations (confidence == 3)
- Conservative: prefetches one stride ahead only
- Backpressure-aware: waits for data cache acknowledgment before issuing next prefetch

### 3.5 Bus Infrastructure

- **AXI4-Lite interconnect:** Shared bus, single outstanding transaction
- **11 AXI masters:** 5 I-cache ports + 5 data cache ports + 1 MAC accelerator
- **5 AXI slaves:** Boot ROM, Main RAM, VGA region, MMIO, MAC
- **QoS arbiter:** Priority + age (200-cycle starvation threshold) + round-robin tie-breaking
- **Dynamic QoS:** Thermal-aware adjustment per core via `QoS_Adjuster.sv`
- **Lock support:** For atomic LR/SC transactions

### 3.6 Boot ROM and RAM

- **Boot ROM:** `Boot_ROM.sv` with `$readmemh` boot image loading
- **Main RAM:** 64KB baseline, parameterized sizing, byte-strobe support, `$readmemh` preloading

## 4. Thermal Management Pipeline

### 4.1 Thermal Sensor (`Thermal_Sensor.sv`)

- 3-node compact RC thermal model per core (Junction → Spreader → Package)
- PVT sensor variation: per-core bias offset + configurable noise floor
- LFSR-based Gaussian noise generator for sensor noise simulation
- For FPGA: current branch keeps thermal sensing in active Phase 7.5 RTL; external XADC-fused telemetry is deferred to future Phase 8 work.

### 4.2 Kalman Filter (`Kalman_Predictor.sv`)

- Scalar Kalman filter per core with Q16.16 fixed-point arithmetic
- Predict step: `T_pred = F * T_est + B * power_proxy + G * T_amb`
- Update step: `K = P_pred / (P_pred + R)`, `T_est = T_pred + K * innovation`
- Per-core measurement noise R calibration (PVT-aware)
- Outputs: `T_estimated`, `T_predicted`, `P_uncertainty`, `thermal_state`, `innovation`

### 4.3 Phase Detector (`Phase_Detector.sv`)

- EMA-based workload phase classification per core
- 4 phases: COMPUTE_BOUND, MEMORY_BOUND, BALANCED, IDLE
- Hysteresis: phase must persist for M samples before classification changes
- `phase_changed` pulse output for proactive migration triggers

### 4.4 Thermal Controller (`Thermal_Controller.sv`)

- **Layer 1 (Safety):** Hard thermal cap, zero-latency combinational path from sensor to `clk_en`
- **Layer 2 (Optimization):** Four selectable operating modes (Baseline/Reactive/Predictive/Full System)
- Clock throttling: 100%, 75%, 50%, 25% duty-cycling per core
- Power budget allocation and redistribution between cores

### 4.5 Dynamic QoS (`QoS_Adjuster.sv`)

- Dynamically adjusts per-core bus QoS based on thermal state
- NORMAL: default QoS; WARNING: QoS-1; CRITICAL: QoS-2
- Cool cores get promoted QoS to reclaim bandwidth

## 5. Task Migration and Power Management

### 5.1 Migration Controller (`Migration_Controller.sv`)

- State machine for register file transfer (x1–x31) between cores
- CSR transfer slots for critical state migration
- PC save/restore with pipeline drain coordination
- Migration cycle/count telemetry

### 5.2 Migration Policy (`Migration_Policy.sv`)

- Cost-benefit analysis: migrate only if `benefit > cost + hysteresis_margin`
- Headroom-based destination selection
- Phase-transition-triggered proactive migration
- Cooldown guard (minimum 10,000 cycles between migrations)
- All-cores-hot blocking, dual-hot imbalance gating

### 5.3 Power Gate Controller (`Power_Gate_Controller.sv`)

- 4-state FSM per core: ACTIVE → THROTTLED → CLOCK_GATED → POWER_GATED
- Migration-trigger wake with configurable latency
- Per-core gated cycle counters for power estimation

### 5.4 Cache Warmer (`Cache_Warmer.sv`)

- Migration-time warm-up window with cycle/count telemetry
- Resume waits for warm-done handshake

### 5.5 Evaluation Framework (`Eval_Framework.sv`)

- Mode-selectable metrics collection (4 thermal management modes)
- Mode select path supports both MMIO writes (`0x9000_0070`) and FPGA D-pad button events (`BTNC/BTNU/BTNL/BTNR` -> modes `0/1/2/3`)
- MMIO-visible run metrics: total cycles, retired instructions, peak/avg temperature, throttle events/cycles, migration count/cycles, power gate cycles, thermal violations, energy estimate
- Workload completion signal freezes metrics for collection

## 6. Peripheral and Accelerator Blocks

### 6.1 Implemented (in active build)

- `Timer.sv` — `mtime`/`mtimecmp` style 64-bit timer with IRQ
- `UART.sv` — TX/RX with configurable baud rate and FIFOs
- `Seven_Seg_Driver.sv` — Real-time thermal dashboard display
- `Boot_ROM.sv` — Boot image source
- `MAC_Unit.sv` — Multiply-accumulate accelerator (Q16.16 fixed-point, Newton-Raphson reciprocal)

### 6.2 Future Phase 8 Additions (not present in this branch)

- `CLINT.sv` — Core-local interruptor for timer + IPI
- `PLIC.sv` — Platform-level interrupt controller
- `SPI_Controller.sv` — SPI master for SD card / flash
- `MMU.sv`, `TLB.sv`, `PTW.sv` — Sv32 virtual memory

## 7. Diagnostics and Signoff

- **Primary script:** `scripts/diagnose_execution.tcl` — static checks, timing/signoff, FPGA diagnostics, SysMon logging
- **Phase 7 regression:** `scripts/run_phase7_regression.ps1` — one-command compile + regression suite
- **FPGA constraints:** `nexys4ddr.xdc` — Nexys 4 DDR pin mapping and clock constraints
- **Thermal simulation:** `scripts/thermal_sim.m` — MATLAB thermal model reference

## 8. File Structure

### Active Build Manifest (`files.f`) — 36 files, 8,659 lines

| Directory | Files | Lines | Key Modules |
|---|---|---|---|
| `rtl/` | 1 | 1,731 | `Orionrv.sv` (top-level SoC) |
| `rtl/core/` | 11 | 2,619 | `RV32_Core.sv` (932), `Pipelined_ALU.sv` (347), `CSR_Unit.sv` (290), `Decoder.sv` (278), `RVC_Decompressor.sv` (210), `Branch_Predictor.sv` (193), `Hazard_Unit.sv` (120), `Loop_Buffer.sv` (102), `Register_File.sv` (58), `Imm_Gen.sv` (48), `Branch_Unit.sv` (41) |
| `rtl/cache/` | 4 | 996 | `Data_Cache.sv` (507), `Instruction_Cache.sv` (256), `Victim_Buffer.sv` (128), `Stride_Prefetcher.sv` (105) |
| `rtl/bus/` | 5 | 1,137 | `AXI_Interconnect.sv` (445), `QoS_Arbiter.sv` (331), `AXI_Master_Wrapper.sv` (170), `AXI_Slave_Wrapper.sv` (152), `Address_Decoder.sv` (39) |
| `rtl/thermal/` | 10 | 1,449 | `Migration_Controller.sv` (255), `Migration_Policy.sv` (216), `Kalman_Predictor.sv` (200), `MAC_Unit.sv` (175 — in accelerator/), `Thermal_Sensor.sv` (163), `Eval_Framework.sv` (168), `Power_Gate_Controller.sv` (161), `Phase_Detector.sv` (130), `Thermal_Controller.sv` (124), `Cache_Warmer.sv` (79), `QoS_Adjuster.sv` (54) |
| `rtl/peripherals/` | 4 | 451 | `Seven_Seg_Driver.sv` (131), `UART.sv` (128), `Timer.sv` (111), `Boot_ROM.sv` (81) |
| `rtl/accelerator/` | 1 | 175 | `MAC_Unit.sv` |

### Testbench Suite (`tb/`) — 20 files

| File | Target Module | Status |
|---|---|---|
| `tb_branch_predictor.sv` | Branch_Predictor | PASS |
| `tb_cache_warmer.sv` | Cache_Warmer | PASS |
| `tb_data_cache.sv` | Data_Cache | PASS |
| `tb_eval_framework.sv` | Eval_Framework | PASS |
| `tb_hazard_unit.sv` | Hazard_Unit | PASS |
| `tb_qos_arbiter.sv` | QoS_Arbiter | PASS |
| `tb_csr_unit.sv` | CSR_Unit | PASS |
| `tb_pipelined_alu.sv` | Pipelined_ALU | PASS |
| `tb_migration_controller.sv` | Migration_Controller | PASS |
| `tb_migration_policy.sv` | Migration_Policy | PASS |
| `tb_instruction_cache.sv` | Instruction_Cache | PASS |
| `tb_kalman_filter.sv` | Kalman_Predictor | PASS |
| `tb_multicore.sv` | Prolepsis multicore integration | PASS |
| `tb_orionrv_integration.sv` | Prolepsis full integration | PASS |
| `tb_phase_detector.sv` | Phase_Detector | PASS |
| `tb_power_gate_controller.sv` | Power_Gate_Controller | PASS |
| `tb_rv32_core.sv` | RV32_Core | PASS |
| `tb_rvc_decompressor.sv` | RVC_Decompressor | PASS |
| `tb_stride_prefetcher.sv` | Stride_Prefetcher | PASS |
| `tb_victim_buffer.sv` | Victim_Buffer | PASS |

## 9. Current Gaps / Next Milestones

- Phase 7.5 microarchitectural upgrades: **implemented** in RTL, pending synthesis verification for timing and area.
- Continue FPGA signoff closure: post-Phase 7.5 synthesis, timing closure verification, and board-level validation.
- Pending Phase 7 items: VGA framebuffer, physical UART/bootloader bring-up, DOOM port, performance profiling.
- Phase 8 Linux/OpenSBI/MMU/PLIC/CLINT work is intentionally not started in this branch and requires explicit user instruction.

## 10. Design Standards and Signoff Criteria

The project cannot be called release-ready unless all gates below pass with archived evidence.

| Gate | Standard | Pass Criteria | Required Evidence |
|---|---|---|---|
| G1 | ISA compliance | All implemented RV32I/RV32M/RV32A/RV32C tests pass; no unexpected traps | ISA test logs and pass summary in `reports/` |
| G2 | Constrained-random + coverage | Randomized regression completes with no fails; functional coverage meets target for hazards, control flow, cache, interrupts, atomics | Regression log + coverage report |
| G3 | Assertion-based protocol checks | No AXI/assertion violations in simulation and stress regressions | Assertion report bundle |
| G4 | Formal verification on critical blocks | Proven properties for reset safety, interconnect liveness/no deadlock, and arbiter fairness (or documented bounded proofs) | Formal run report + property list |
| G5 | CDC/RDC cleanliness | No unreviewed critical CDC/RDC violations; all waivers justified | CDC/RDC report + waiver file |
| G6 | Constraint completeness | No unconstrained endpoints/clocks; I/O timing constraints reviewed for board interfaces | `check_timing` and constraints report |
| G7 | Timing closure with margin | Setup and hold meet timing in signoff build; target positive slack margin documented for repeatability | Timing summary + top path report |
| G8 | On-chip observability | Runtime counters/flags for retirement, stalls, watchdog cause, and thermal events are readable and validated | MMIO map + hardware readback log |
| G9 | Built-in self-test | Power-on sanity tests for RAM/MMIO/interrupt path complete with deterministic pass/fail code | BIST design note + board run log |
| G10 | Thermal and power qualification | Threshold behavior (warn/L1/crit) and telemetry path verified under stress workloads | Thermal run logs + threshold event trace |
| G11 | Long-run stability | Burn-in run completes with no hangs, deadlocks, or unbounded error counters | 12-24h stability report |
| G12 | Automated CI regression | CI pipeline gates lint, simulation, formal subset, synthesis, and report archiving for each candidate build | CI config + latest pipeline artifacts |

### 10.1 Definition of "Production-Ready" for Prolepsis

Prolepsis is considered production-ready only when G1 through G12 are all green on the same tagged revision and all evidence artifacts are retained with that release.
