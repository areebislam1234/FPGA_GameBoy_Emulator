
`timescale 1ns / 1ps
//=============================================================================
// wram.v
// 
//
// Generic synchronous read/write RAM. Serves double duty for both WRAM
// and HRAM in this project -- same module, instantiated twice at the top
// level with different ADDR_WIDTH:
//
//     wram #(.ADDR_WIDTH(13)) u_wram (...);  // 8192 entries, C000-DFFF
//     wram #(.ADDR_WIDTH(7))  u_hram (...);  // 128 entries (FF80-FFFE only
//                                            // uses 127 of them -- leaving
//                                            // SIZE at its default of
//                                            // 2**ADDR_WIDTH rather than
//                                            // trimming it to exactly 127
//                                            // costs one unused byte of BRAM
//                                            // but makes an out-of-range
//                                            // address structurally
//                                            // impossible instead of merely
//                                            // unlikely)
//
// Connects directly to address_decoder.v's wram_*/hram_* ports -- widths
// already match (wram_addr is [12:0] = 13 bits, hram_addr is [6:0] = 7
// bits).
//
// Synchronous read AND write, same BRAM-inference pattern as rom.v --
// reads have 1 cycle of latency, matching every other region
// address_decoder.v expects.
//
// Read-during-write behavior: if `we` and a read of the same address land
// in the same cycle, dout returns the OLD value (the one being
// overwritten), not the new one -- "read-first," one of Xilinx's directly
// supported BRAM modes, and the safest default absent a specific reason
// to need same-cycle write-to-read forwarding. In practice the SM83 never
// reads and writes the same address within one machine cycle anyway
// (e.g. INC (HL) reads, then writes, across two separate cycles), so this
// choice doesn't change CPU-visible behavior here -- it's a deliberate,
// predictable default rather than leaving it to whatever a synthesis
// tool happens to infer.
//=============================================================================
module wram #(
    parameter ADDR_WIDTH = 13,               // -> 2**ADDR_WIDTH addressable entries
    parameter SIZE       = (1 << ADDR_WIDTH) // usable depth; see note above on HRAM
) (
    input  wire                  clk,
    input  wire [ADDR_WIDTH-1:0] addr,
    input  wire [7:0]            din,
    input  wire                  we,
    output reg  [7:0]            dout
);
 
    reg [7:0] mem [0:SIZE-1];
 
    always @(posedge clk) begin
        if (we)
            mem[addr] <= din;
        dout <= mem[addr];   // reads the PRE-write value -- read-first, see header
    end
 
endmodule