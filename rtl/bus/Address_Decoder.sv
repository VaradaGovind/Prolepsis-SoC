`timescale 1ns / 1ps
//============================================================================
// OrionRV - Address Decoder
//
// Routes transactions to the correct slave port based on the address map
// defined in axi_pkg.sv. Returns DECERR for unmapped addresses.
//
// UART and Timer are sub-decoded within the MMIO region (0x9000_xxxx)
// by the top-level module, not by this decoder.
//============================================================================

module address_decoder (
    input  logic [31:0] addr,
    output logic  [2:0]  slave_sel,    // Slave port index
    output logic         valid         // 1 = address maps to a valid slave
);

    always_comb begin
        valid     = 1'b1;
        slave_sel = 3'd0;

        if ((addr & axi_pkg::ADDR_BOOT_MASK) == axi_pkg::ADDR_BOOT_BASE)
            slave_sel = axi_pkg::SLAVE_BOOT;
        else if ((addr & axi_pkg::ADDR_RAM_MASK) == axi_pkg::ADDR_RAM_BASE)
            slave_sel = axi_pkg::SLAVE_RAM;
        else if ((addr & axi_pkg::ADDR_VGA_MASK) == axi_pkg::ADDR_VGA_BASE)
            slave_sel = axi_pkg::SLAVE_VGA;
        else if ((addr & axi_pkg::ADDR_MMIO_MASK) == axi_pkg::ADDR_MMIO_BASE)
            slave_sel = axi_pkg::SLAVE_MMIO;
        else if ((addr & axi_pkg::ADDR_MAC_MASK) == axi_pkg::ADDR_MAC_BASE)
            slave_sel = axi_pkg::SLAVE_MAC;
        else begin
            slave_sel = 3'd0;
            valid     = 1'b0;
        end
    end

endmodule
