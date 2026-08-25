$ErrorActionPreference = 'Stop'

$regs = @{
    'x0' = 0;  'zero' = 0;
    'ra' = 1;  'sp' = 2;  'gp' = 3;  'tp' = 4;
    't0' = 5;  't1' = 6;  't2' = 7;
    's0' = 8;  'fp' = 8;  's1' = 9;
    'a0' = 10; 'a1' = 11; 'a2' = 12; 'a3' = 13;
    'a4' = 14; 'a5' = 15; 'a6' = 16; 'a7' = 17;
    's2' = 18; 's3' = 19; 's4' = 20; 's5' = 21;
    's6' = 22; 's7' = 23; 's8' = 24; 's9' = 25; 's10' = 26; 's11' = 27;
    't3' = 28; 't4' = 29; 't5' = 30; 't6' = 31;
}

$csrs = @{
    'mstatus' = 0x300;
    'mie'     = 0x304;
    'mtvec'   = 0x305;
    'mcause'  = 0x342;
}

function Enc-R([int]$funct7, [int]$rs2, [int]$rs1, [int]$funct3, [int]$rd, [int]$opcode) {
    return (($funct7 -band 0x7F) -shl 25) -bor
           (($rs2    -band 0x1F) -shl 20) -bor
           (($rs1    -band 0x1F) -shl 15) -bor
           (($funct3 -band 0x07) -shl 12) -bor
           (($rd     -band 0x1F) -shl 7)  -bor
           ($opcode  -band 0x7F)
}

function Enc-I([int]$imm, [int]$rs1, [int]$funct3, [int]$rd, [int]$opcode) {
    $u = $imm -band 0xFFF
    return (($u      -band 0xFFF) -shl 20) -bor
           (($rs1    -band 0x1F)  -shl 15) -bor
           (($funct3 -band 0x07)  -shl 12) -bor
           (($rd     -band 0x1F)  -shl 7)  -bor
           ($opcode  -band 0x7F)
}

function Enc-S([int]$imm, [int]$rs2, [int]$rs1, [int]$funct3, [int]$opcode) {
    $u = $imm -band 0xFFF
    $imm11_5 = ($u -shr 5) -band 0x7F
    $imm4_0  = $u -band 0x1F
    return ($imm11_5 -shl 25) -bor
           (($rs2    -band 0x1F) -shl 20) -bor
           (($rs1    -band 0x1F) -shl 15) -bor
           (($funct3 -band 0x07) -shl 12) -bor
           ($imm4_0  -shl 7)      -bor
           ($opcode  -band 0x7F)
}

function Enc-B([int]$imm, [int]$rs2, [int]$rs1, [int]$funct3, [int]$opcode) {
    if (($imm % 2) -ne 0) {
        throw "B-type offset not halfword aligned: $imm"
    }
    $u = $imm -band 0x1FFF
    $bit12    = ($u -shr 12) -band 0x1
    $bit11    = ($u -shr 11) -band 0x1
    $bits10_5 = ($u -shr 5)  -band 0x3F
    $bits4_1  = ($u -shr 1)  -band 0x0F
    return ($bit12 -shl 31) -bor
           ($bits10_5 -shl 25) -bor
           (($rs2 -band 0x1F) -shl 20) -bor
           (($rs1 -band 0x1F) -shl 15) -bor
           (($funct3 -band 0x07) -shl 12) -bor
           ($bits4_1 -shl 8) -bor
           ($bit11 -shl 7) -bor
           ($opcode -band 0x7F)
}

function Enc-U([int]$imm20, [int]$rd, [int]$opcode) {
    return (($imm20 -band 0xFFFFF) -shl 12) -bor
           (($rd    -band 0x1F)    -shl 7)  -bor
           ($opcode -band 0x7F)
}

function Enc-J([int]$imm, [int]$rd, [int]$opcode) {
    if (($imm % 2) -ne 0) {
        throw "J-type offset not halfword aligned: $imm"
    }
    $u = $imm -band 0x1FFFFF
    $bit20     = ($u -shr 20) -band 0x1
    $bits19_12 = ($u -shr 12) -band 0xFF
    $bit11     = ($u -shr 11) -band 0x1
    $bits10_1  = ($u -shr 1)  -band 0x3FF

    return ($bit20 -shl 31) -bor
           ($bits19_12 -shl 12) -bor
           ($bit11 -shl 20) -bor
           ($bits10_1 -shl 21) -bor
           (($rd -band 0x1F) -shl 7) -bor
           ($opcode -band 0x7F)
}

$prog = @(
    @{ label = '_start'; op = 'auipc'; rd = 't0'; imm = 0 },
    @{ op = 'addi'; rd = 't0'; rs1 = 't0'; target = 'trap_handler' },
    @{ op = 'csrw'; csr = 'mtvec'; rs1 = 't0' },
    @{ op = 'lui'; rd = 's0'; imm20 = 0x20000 },
    @{ op = 'lui'; rd = 's1'; imm20 = 0x90000 },
    @{ op = 'addi'; rd = 't0'; rs1 = 'x0'; imm = 3 },
    @{ op = 'sw'; rs1 = 's1'; rs2 = 't0'; imm = 0x70 },
    @{ op = 'addi'; rd = 's2'; rs1 = 'x0'; imm = 0 },
    @{ op = 'addi'; rd = 's3'; rs1 = 'x0'; imm = 1 },
    @{ op = 'lui'; rd = 's4'; imm20 = 0x12345 },
    @{ op = 'addi'; rd = 's4'; rs1 = 's4'; imm = 0x678 },
    @{ op = 'lui'; rd = 's5'; imm20 = 0x89ABD },
    @{ op = 'addi'; rd = 's5'; rs1 = 's5'; imm = -529 },
    @{ op = 'addi'; rd = 's6'; rs1 = 'x0'; imm = 0 },

    @{ label = 'main_loop'; op = 'slli'; rd = 't0'; rs1 = 's2'; shamt = 8 },
    @{ op = 'slli'; rd = 't1'; rs1 = 's3'; shamt = 12 },
    @{ op = 'or'; rd = 't0'; rs1 = 't0'; rs2 = 't1' },
    @{ op = 'addi'; rd = 't0'; rs1 = 't0'; imm = 1 },
    @{ op = 'sw'; rs1 = 's1'; rs2 = 't0'; imm = 0x50 },

    @{ label = 'wait_busy_high'; op = 'lw'; rd = 't2'; rs1 = 's1'; imm = 0x54 },
    @{ op = 'andi'; rd = 't3'; rs1 = 't2'; imm = 8 },
    @{ op = 'bne'; rs1 = 't3'; rs2 = 'x0'; target = 'wait_busy_low' },
    @{ op = 'jal'; rd = 'x0'; target = 'wait_busy_high' },

    @{ label = 'wait_busy_low'; op = 'lw'; rd = 't2'; rs1 = 's1'; imm = 0x54 },
    @{ op = 'andi'; rd = 't3'; rs1 = 't2'; imm = 8 },
    @{ op = 'bne'; rs1 = 't3'; rs2 = 'x0'; target = 'wait_busy_low' },

    @{ op = 'addi'; rd = 't0'; rs1 = 'x0'; imm = 2 },
    @{ op = 'sw'; rs1 = 's1'; rs2 = 't0'; imm = 0x50 },

    @{ op = 'lw'; rd = 'a0'; rs1 = 's1'; imm = 0x58 },
    @{ op = 'lw'; rd = 'a1'; rs1 = 's1'; imm = 0x5C },
    @{ op = 'lw'; rd = 'a6'; rs1 = 's1'; imm = 0x68 },
    @{ op = 'xor'; rd = 'a2'; rs1 = 'a0'; rs2 = 'a1' },
    @{ op = 'xor'; rd = 'a2'; rs1 = 'a2'; rs2 = 'a6' },
    @{ op = 'sw'; rs1 = 's1'; rs2 = 'a2'; imm = 0x00 },

    @{ op = 'add'; rd = 't0'; rs1 = 's4'; rs2 = 's5' },
    @{ op = 'xor'; rd = 't1'; rs1 = 's4'; rs2 = 's5' },
    @{ op = 'mul'; rd = 'a3'; rs1 = 's4'; rs2 = 's5' },
    @{ op = 'mulh'; rd = 'a4'; rs1 = 's4'; rs2 = 's5' },
    @{ op = 'srl'; rd = 'a5'; rs1 = 'a3'; rs2 = 's3' },
    @{ op = 'add'; rd = 's4'; rs1 = 't0'; rs2 = 'a4' },
    @{ op = 'xor'; rd = 's5'; rs1 = 't1'; rs2 = 'a5' },
    @{ op = 'andi'; rd = 't4'; rs1 = 's4'; imm = 1 },
    @{ op = 'bne'; rs1 = 't4'; rs2 = 'x0'; target = 'mix_odd' },
    @{ op = 'add'; rd = 's5'; rs1 = 's5'; rs2 = 's4' },
    @{ op = 'jal'; rd = 'x0'; target = 'mix_join' },

    @{ label = 'mix_odd'; op = 'xor'; rd = 's5'; rs1 = 's5'; rs2 = 's4' },

    @{ label = 'mix_join'; op = 'addi'; rd = 's2'; rs1 = 's2'; imm = 1 },
    @{ op = 'addi'; rd = 't5'; rs1 = 'x0'; imm = 5 },
    @{ op = 'blt'; rs1 = 's2'; rs2 = 't5'; target = 'src_ok' },
    @{ op = 'addi'; rd = 's2'; rs1 = 'x0'; imm = 0 },

    @{ label = 'src_ok'; op = 'addi'; rd = 's3'; rs1 = 's2'; imm = 1 },
    @{ op = 'blt'; rs1 = 's3'; rs2 = 't5'; target = 'dst_ok' },
    @{ op = 'addi'; rd = 's3'; rs1 = 'x0'; imm = 0 },

    @{ label = 'dst_ok'; op = 'addi'; rd = 's6'; rs1 = 's6'; imm = 1 },
    @{ op = 'andi'; rd = 't6'; rs1 = 's6'; imm = 15 },
    @{ op = 'bne'; rs1 = 't6'; rs2 = 'x0'; target = 'main_loop' },

    @{ op = 'addi'; rd = 't0'; rs1 = 'x0'; imm = 1 },
    @{ op = 'sw'; rs1 = 's1'; rs2 = 't0'; imm = 0xC0 },
    @{ op = 'jal'; rd = 'x0'; target = 'main_loop' },

    @{ op = 'nop' }, @{ op = 'nop' }, @{ op = 'nop' },

    @{ label = 'trap_handler'; op = 'csrr'; rd = 't0'; csr = 'mcause' },
    @{ op = 'lui'; rd = 't1'; imm20 = 0x80000 },
    @{ op = 'addi'; rd = 't1'; rs1 = 't1'; imm = 0x10 },
    @{ op = 'bne'; rs1 = 't0'; rs2 = 't1'; target = 'trap_exit' },
    @{ op = 'lui'; rd = 't3'; imm20 = 0x10 },
    @{ label = 'cool_down'; op = 'addi'; rd = 't3'; rs1 = 't3'; imm = -1 },
    @{ op = 'bne'; rs1 = 't3'; rs2 = 'x0'; target = 'cool_down' },
    @{ label = 'trap_exit'; op = 'mret' }
)

$labels = @{}
$pc = 0
foreach ($ins in $prog) {
    if ($ins.ContainsKey('label')) {
        $labels[$ins.label] = $pc
    }
    $pc += 4
}

$words = New-Object System.Collections.Generic.List[UInt32]
$pc = 0
foreach ($ins in $prog) {
    $w = 0
    switch ($ins.op) {
        'nop'   { $w = Enc-I 0 $regs['x0'] 0 $regs['x0'] 0x13 }
        'auipc' { $w = Enc-U $ins.imm $regs[$ins.rd] 0x17 }
        'lui'   { $w = Enc-U $ins.imm20 $regs[$ins.rd] 0x37 }

        'addi'  {
            if ($ins.ContainsKey('target')) {
                # Target is absolute, but here auipc + addi gives PC + offset.
                # ADDI follows AUIPC in this pattern, so use AUIPC PC base explicitly.
                $auipc_pc = $pc - 4
                $off = $labels[$ins.target] - $auipc_pc
                $w = Enc-I $off $regs[$ins.rs1] 0 $regs[$ins.rd] 0x13
            } else {
                $w = Enc-I $ins.imm $regs[$ins.rs1] 0 $regs[$ins.rd] 0x13
            }
        }
        'andi'  { $w = Enc-I $ins.imm $regs[$ins.rs1] 7 $regs[$ins.rd] 0x13 }
        'xori'  { $w = Enc-I $ins.imm $regs[$ins.rs1] 4 $regs[$ins.rd] 0x13 }
        'lw'    { $w = Enc-I $ins.imm $regs[$ins.rs1] 2 $regs[$ins.rd] 0x03 }
        'sw'    { $w = Enc-S $ins.imm $regs[$ins.rs2] $regs[$ins.rs1] 2 0x23 }

        'slli'  { $w = Enc-I ($ins.shamt -band 0x1F) $regs[$ins.rs1] 1 $regs[$ins.rd] 0x13 }
        'srli'  { $w = Enc-I ($ins.shamt -band 0x1F) $regs[$ins.rs1] 5 $regs[$ins.rd] 0x13 }

        'add'   { $w = Enc-R 0x00 $regs[$ins.rs2] $regs[$ins.rs1] 0 $regs[$ins.rd] 0x33 }
        'sub'   { $w = Enc-R 0x20 $regs[$ins.rs2] $regs[$ins.rs1] 0 $regs[$ins.rd] 0x33 }
        'xor'   { $w = Enc-R 0x00 $regs[$ins.rs2] $regs[$ins.rs1] 4 $regs[$ins.rd] 0x33 }
        'or'    { $w = Enc-R 0x00 $regs[$ins.rs2] $regs[$ins.rs1] 6 $regs[$ins.rd] 0x33 }
        'and'   { $w = Enc-R 0x00 $regs[$ins.rs2] $regs[$ins.rs1] 7 $regs[$ins.rd] 0x33 }
        'sll'   { $w = Enc-R 0x00 $regs[$ins.rs2] $regs[$ins.rs1] 1 $regs[$ins.rd] 0x33 }
        'srl'   { $w = Enc-R 0x00 $regs[$ins.rs2] $regs[$ins.rs1] 5 $regs[$ins.rd] 0x33 }

        'mul'   { $w = Enc-R 0x01 $regs[$ins.rs2] $regs[$ins.rs1] 0 $regs[$ins.rd] 0x33 }
        'mulh'  { $w = Enc-R 0x01 $regs[$ins.rs2] $regs[$ins.rs1] 1 $regs[$ins.rd] 0x33 }
        'div'   { $w = Enc-R 0x01 $regs[$ins.rs2] $regs[$ins.rs1] 4 $regs[$ins.rd] 0x33 }
        'rem'   { $w = Enc-R 0x01 $regs[$ins.rs2] $regs[$ins.rs1] 6 $regs[$ins.rd] 0x33 }

        'blt'   {
            $off = $labels[$ins.target] - $pc
            $w = Enc-B $off $regs[$ins.rs2] $regs[$ins.rs1] 4 0x63
        }
        'bne'   {
            $off = $labels[$ins.target] - $pc
            $w = Enc-B $off $regs[$ins.rs2] $regs[$ins.rs1] 1 0x63
        }
        'jal'   {
            $off = $labels[$ins.target] - $pc
            $w = Enc-J $off $regs[$ins.rd] 0x6F
        }

        'csrw'  {
            $w = (($csrs[$ins.csr] -band 0xFFF) -shl 20) -bor
                 (($regs[$ins.rs1] -band 0x1F) -shl 15) -bor
                 (1 -shl 12) -bor
                 (($regs['x0'] -band 0x1F) -shl 7) -bor
                 0x73
        }
        'csrs'  {
            $w = (($csrs[$ins.csr] -band 0xFFF) -shl 20) -bor
                 (($regs[$ins.rs1] -band 0x1F) -shl 15) -bor
                 (2 -shl 12) -bor
                 (($regs['x0'] -band 0x1F) -shl 7) -bor
                 0x73
        }
        'csrr'  {
            $w = (($csrs[$ins.csr] -band 0xFFF) -shl 20) -bor
                 (($regs['x0'] -band 0x1F) -shl 15) -bor
                 (2 -shl 12) -bor
                 (($regs[$ins.rd] -band 0x1F) -shl 7) -bor
                 0x73
        }
        'mret'  { $w = 0x30200073 }

        default { throw "Unknown op: $($ins.op)" }
    }

    $u32 = [System.BitConverter]::ToUInt32([System.BitConverter]::GetBytes([int]$w), 0)
    $words.Add($u32) | Out-Null
    $pc += 4
}

$out = New-Object System.Collections.Generic.List[string]
$out.Add('@00000000') | Out-Null
foreach ($w in $words) {
    $out.Add(('{0:X8}' -f $w)) | Out-Null
}

Set-Content -Path (Join-Path $PSScriptRoot '..\sw\boot.hex') -Value $out -NoNewline:$false
Set-Content -Path (Join-Path $PSScriptRoot '..\boot.hex') -Value $out -NoNewline:$false
Write-Output ("Generated {0} words" -f $words.Count)
Write-Output ("trap_handler @ 0x{0:X8}" -f $labels['trap_handler'])
