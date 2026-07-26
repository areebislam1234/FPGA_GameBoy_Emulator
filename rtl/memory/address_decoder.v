//======================================================
// Game Boy Address Decoder
//
// Determines which memory device should respond
// to the CPU's current address.
//
// Author: Frankie Chong
//======================================================

`timescale 1ns / 1ps
//=============================================================================
// addr_decoder.v
// 
//
// Top-level address decoder / bus router for the Game Boy's 16-bit memory
// map. This module is pure routing logic -- it does NOT contain any
// storage itself (except the two tiny interrupt registers noted below).
// Real ROM / RAM / VRAM / OAM modules connect to the external ports and
// are instantiated one level up, in the top-level bus module.
//
// Memory map (see Pan Docs: https://gbdev.io/pandocs/Memory_Map.html):
//   0000-7FFF  ROM                 32 KiB, read-only here (no MBC -- Tetris)
//   8000-9FFF  VRAM                8 KiB
//   A000-BFFF  External cart RAM   8 KiB (unused by Tetris, wired anyway)
//   C000-DFFF  WRAM                8 KiB
//   E000-FDFF  Echo RAM            mirrors C000-DDFF
//   FE00-FE9F  OAM                 160 B
//   FEA0-FEFF  Unusable            reads as 0x00
//   FF00-FF7F  I/O registers       FF0F (IF) handled internally, rest external
//   FF80-FFFE  HRAM                127 B
//   FFFF       IE                  1 B, handled internally
//
// ---------------------------------------------------------------------------
// TIMING CONTRACT (read this before wiring up the CPU or any memory):
//
// ROM/VRAM/ExtRAM/WRAM/OAM are assumed to be synchronous-read block RAMs,
// matching how Xilinx BRAM actually infers on the Basys3:
//
//     always @(posedge clk) if (en) dout <= mem[addr];
//
// That means dout from those modules is valid the cycle AFTER addr is
// presented -- one cycle of read latency, not zero. This decoder accounts
// for that: `region_r` is a REGISTERED copy of which region was selected,
// so the read-data mux at the bottom picks the right (now-valid) dout one
// cycle later, in lockstep with the BRAMs. If you swap in a combinational
// (asynchronous-read) memory anywhere, this mux will be off by a cycle for
// that region -- keep everything synchronous-read.
//
// I/O and HRAM are left as external ports too so their owners (joypad,
// timer, sound, LCD control, etc.) can be simple combinational or
// synchronous registers as appropriate -- this decoder doesn't assume
// either way for them, it just qualifies write-enables and mux-selects.
//=============================================================================
module addr_decoder (
    input  wire        clk,
    input  wire        rst,

    // ---- CPU-facing bus -----------------------------------------------
    input  wire [15:0] addr,
    input  wire [7:0]  din,
    input  wire        we,
    output reg  [7:0]  dout,

    // ---- ROM (read-only, no write port) --------------------------------
    output wire [14:0] rom_addr,
    input  wire [7:0]  rom_dout,

    // ---- External cartridge RAM ----------------------------------------
    output wire [12:0] extram_addr,
    output wire [7:0]  extram_din,
    output wire        extram_we,
    input  wire [7:0]  extram_dout,

    // ---- VRAM (owned by /rtl/video/) -----------------------------------
    output wire [12:0] vram_addr,
    output wire [7:0]  vram_din,
    output wire        vram_we,
    input  wire [7:0]  vram_dout,

    // ---- WRAM (also serves Echo RAM, remapped below) -------------------
    output wire [12:0] wram_addr,
    output wire [7:0]  wram_din,
    output wire        wram_we,
    input  wire [7:0]  wram_dout,

    // ---- OAM (owned by /rtl/video/) -------------------------------------
    output wire [7:0]  oam_addr,
    output wire [7:0]  oam_din,
    output wire        oam_we,
    input  wire [7:0]  oam_dout,

    // ---- Generic I/O region FF00-FF7F, excluding FF0F/IF ----------------
    output wire [6:0]  io_addr,
    output wire [7:0]  io_din,
    output wire        io_we,
    input  wire [7:0]  io_dout,

    // ---- HRAM -------------------------------------------------------------
    output wire [6:0]  hram_addr,
    output wire [7:0]  hram_din,
    output wire        hram_we,
    input  wire [7:0]  hram_dout,

    // ---- Direct (non-bus) access for the CPU's interrupt logic ----------
    // IE (FFFF) and IF (FF0F) are small enough, and central enough, that
    // they live directly in this module rather than an external one. They
    // are still readable/writable via the normal bus (below) AND exposed
    // here as plain wires so Person A's interrupt-dispatch logic doesn't
    // have to do a full bus read every time it needs to check them.
    output wire [7:0]  ie_reg,
    output wire [7:0]  if_reg
);

    // ---- Region decode (combinational) ---------------------------------
    wire sel_rom      = (addr <= 16'h7FFF);
    wire sel_vram     = (addr >= 16'h8000) && (addr <= 16'h9FFF);
    wire sel_extram   = (addr >= 16'hA000) && (addr <= 16'hBFFF);
    wire sel_wram     = (addr >= 16'hC000) && (addr <= 16'hDFFF);
    wire sel_echo     = (addr >= 16'hE000) && (addr <= 16'hFDFF);
    wire sel_oam      = (addr >= 16'hFE00) && (addr <= 16'hFE9F);
    // sel_unusable (FEA0-FEFF) needs no wire -- it just falls through to
    // the default "no region selected" case below.
    wire sel_if       = (addr == 16'hFF0F);
    wire sel_io       = (addr >= 16'hFF00) && (addr <= 16'hFF7F) && !sel_if;
    wire sel_hram     = (addr >= 16'hFF80) && (addr <= 16'hFFFE);
    wire sel_ie       = (addr == 16'hFFFF);

    // ---- Address remap to each region's local (zero-based) offset -----
    assign rom_addr    = addr[14:0];                                   // 0000-7FFF -> 0-7FFF
    assign vram_addr   = addr - 16'h8000;                               // 8000-9FFF -> 0-1FFF
    assign extram_addr = addr - 16'hA000;                               // A000-BFFF -> 0-1FFF
    assign wram_addr   = sel_echo ? (addr - 16'hE000) : (addr - 16'hC000); // -> 0-1FFF
    assign oam_addr    = addr - 16'hFE00;                               // FE00-FE9F -> 00-9F
    assign io_addr      = addr - 16'hFF00;                               // FF00-FF7F -> 00-7F
    assign hram_addr    = addr - 16'hFF80;                               // FF80-FFFE -> 00-7E

    // ---- Data-in passthrough (harmless unless that region's _we fires) --
    assign extram_din = din;
    assign vram_din   = din;
    assign wram_din    = din;
    assign oam_din    = din;
    assign io_din       = din;
    assign hram_din     = din;

    // ---- Write-enable qualification: only the addressed region may write
    assign extram_we = we && sel_extram;
    assign vram_we   = we && sel_vram;
    assign wram_we    = we && (sel_wram || sel_echo);
    assign oam_we    = we && sel_oam;
    assign io_we       = we && sel_io;
    assign hram_we     = we && sel_hram;
    // NOTE: no rom_we -- ROM is read-only in this no-MBC design. Writes to
    // 0000-7FFF are silently dropped (real MBC carts would decode these as
    // banking-control writes; add that here if/when you support MBC1).

    // ---- IE / IF internal registers -------------------------------------
    reg [7:0] ie_reg_i;
    reg [7:0] if_reg_i;
    assign ie_reg = ie_reg_i;
    assign if_reg = if_reg_i;

    always @(posedge clk) begin
        if (rst) begin
            ie_reg_i <= 8'h00;
            if_reg_i <= 8'h00;
        end else begin
            if (we && sel_ie) ie_reg_i <= din;
            if (we && sel_if) if_reg_i <= din;
        end
    end

    // ---- Registered region select, for the read-data mux ----------------
    localparam [3:0] R_ROM=0, R_VRAM=1, R_EXTRAM=2, R_WRAM=3, R_OAM=4,
                      R_IO=5, R_HRAM=6, R_IE=7, R_IF=8, R_NONE=9;

    reg [3:0] region_r;

    always @(posedge clk) begin
        if (rst) begin
            region_r <= R_NONE;
        end else begin
            if (sel_rom)                    region_r <= R_ROM;
            else if (sel_vram)               region_r <= R_VRAM;
            else if (sel_extram)             region_r <= R_EXTRAM;
            else if (sel_wram || sel_echo)   region_r <= R_WRAM;
            else if (sel_oam)                region_r <= R_OAM;
            else if (sel_io)                  region_r <= R_IO;
            else if (sel_hram)                region_r <= R_HRAM;
            else if (sel_ie)                  region_r <= R_IE;
            else if (sel_if)                  region_r <= R_IF;
            else                              region_r <= R_NONE;
        end
    end

    always @(*) begin
        case (region_r)
            R_ROM:    dout = rom_dout;
            R_VRAM:   dout = vram_dout;
            R_EXTRAM: dout = extram_dout;
            R_WRAM:   dout = wram_dout;
            R_OAM:    dout = oam_dout;
            R_IO:     dout = io_dout;
            R_HRAM:   dout = hram_dout;
            R_IE:     dout = ie_reg_i;
            R_IF:     dout = if_reg_i;
            default:  dout = 8'h00;   // unusable region (FEA0-FEFF) or reset
        endcase
    end

endmodule