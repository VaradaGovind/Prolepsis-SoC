// ============================================================================
// OrionRV - 7-Segment Display Driver (Nexys 4 DDR)
//
// Drives the 8-digit multiplexed 7-segment display on the Nexys 4 DDR board.
// Active-low segments (CA-CG) and active-low anodes (AN0-AN7).
//
// Display Layout (Dashboard Mode):
//   [D7][D6][D5][D4] | [D3][D2][D1][D0]
//   Left 4 digits    | Right 4 digits
//   Core Status/Mask | Temperature (hex) or Error Code
//
// Inputs:
//   display_data[31:0] — 32-bit hex value, each nibble maps to one digit
//   dp_mask[7:0]       — Decimal point enable per digit (active-high input)
//   blanking[7:0]      — Blank (turn off) individual digits
//
// The display refreshes at ~1kHz (100MHz / 2^17 ≈ 763 Hz).
// ============================================================================

module Seven_Seg_Driver (
    input  logic        clk,         // 100MHz system clock
    input  logic        rst_n,       // Active-low reset
    
    // Data Inputs
    input  logic [31:0] display_data, // 8 hex digits (D7=MSN, D0=LSN)
    input  logic [7:0]  dp_mask,      // Decimal point per digit (1=on)
    input  logic [7:0]  blanking,     // Blank digits (1=blank)
    
    // 7-Segment Outputs (active-low, accent the Nexys 4 DDR)
    output logic  [6:0]  seg,          // CA, CB, CC, CD, CE, CF, CG
    output logic         dp,           // Decimal point
    output logic  [7:0]  an            // Anode enables (active-low)
);

    // ---------------------------------------------------------------
    // Refresh counter: cycle through 8 digits
    // 100MHz / 2^17 = ~763 Hz per digit, 8 digits = ~95 Hz full refresh
    // ---------------------------------------------------------------
    logic [19:0] refresh_counter;
    
    always_ff @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            refresh_counter <= 20'd0;
        else
            refresh_counter <= refresh_counter + 20'd1;
    end
    
    logic [2:0] digit_sel;
    assign digit_sel = refresh_counter[19:17]; // 3-bit digit selector

    // ---------------------------------------------------------------
    // Anode selector: activate one digit at a time (active-low)
    // ---------------------------------------------------------------
    always_comb begin
        an = 8'b1111_1111; // All off
        an[digit_sel] = 1'b0;    // Turn on selected digit
    end

    // ---------------------------------------------------------------
    // Digit data mux: select the nibble for the active digit
    // ---------------------------------------------------------------
    logic [3:0] current_nibble;
    logic       current_dp;
    logic       current_blank;
    
    always_comb begin
        case (digit_sel)
            3'd0: begin current_nibble = display_data[3:0];   current_dp = dp_mask[0]; current_blank = blanking[0]; end
            3'd1: begin current_nibble = display_data[7:4];   current_dp = dp_mask[1]; current_blank = blanking[1]; end
            3'd2: begin current_nibble = display_data[11:8];  current_dp = dp_mask[2]; current_blank = blanking[2]; end
            3'd3: begin current_nibble = display_data[15:12]; current_dp = dp_mask[3]; current_blank = blanking[3]; end
            3'd4: begin current_nibble = display_data[19:16]; current_dp = dp_mask[4]; current_blank = blanking[4]; end
            3'd5: begin current_nibble = display_data[23:20]; current_dp = dp_mask[5]; current_blank = blanking[5]; end
            3'd6: begin current_nibble = display_data[27:24]; current_dp = dp_mask[6]; current_blank = blanking[6]; end
            3'd7: begin current_nibble = display_data[31:28]; current_dp = dp_mask[7]; current_blank = blanking[7]; end
            default: begin current_nibble = 4'h0; current_dp = 1'b0; current_blank = 1'b1; end
        endcase
    end

    // ---------------------------------------------------------------
    // Hex-to-7-segment decoder (active-low output)
    //
    // Segment mapping:     _a_
    //                    |f   |b
    //                     _g_
    //                    |e   |c
    //                     _d_
    //
    // seg[6:0] = {a, b, c, d, e, f, g}  (active-low)
    // ---------------------------------------------------------------
    logic [6:0] seg_pattern; // Active-HIGH version (inverted at output)
    
    always_comb begin
        case (current_nibble)
            //                  abcdefg
            4'h0: seg_pattern = 7'b1111110;
            4'h1: seg_pattern = 7'b0110000;
            4'h2: seg_pattern = 7'b1101101;
            4'h3: seg_pattern = 7'b1111001;
            4'h4: seg_pattern = 7'b0110011;
            4'h5: seg_pattern = 7'b1011011;
            4'h6: seg_pattern = 7'b1011111;
            4'h7: seg_pattern = 7'b1110000;
            4'h8: seg_pattern = 7'b1111111;
            4'h9: seg_pattern = 7'b1111011;
            4'hA: seg_pattern = 7'b1110111; // A
            4'hB: seg_pattern = 7'b0011111; // b
            4'hC: seg_pattern = 7'b1001110; // C
            4'hD: seg_pattern = 7'b0111101; // d
            4'hE: seg_pattern = 7'b1001111; // E
            4'hF: seg_pattern = 7'b1000111; // F
            default: seg_pattern = 7'b0000000;
        endcase
    end

    // ---------------------------------------------------------------
    // Output: invert for active-low, apply blanking
    // ---------------------------------------------------------------
    always_comb begin
        if (current_blank) begin
            seg = 7'b1111111; // All segments OFF (active-low)
            dp  = 1'b1;       // DP OFF
        end else begin
            // XDC maps seg[0] to CA(a) and seg[6] to CG(g), while seg_pattern
            // is coded as {a,b,c,d,e,f,g}; reverse bit order at the output.
            seg = ~{seg_pattern[0], seg_pattern[1], seg_pattern[2],
                    seg_pattern[3], seg_pattern[4], seg_pattern[5], seg_pattern[6]};
            dp  = ~current_dp;  // Invert for active-low
        end
    end

endmodule
