`timescale 1ns / 1ps
//=============================================================================
// memory_top.v
// 
//
// Integrates everything Phase 4 covers into one block: address_decoder.v
// wired to a real rom.v and two real wram.v instances (WRAM and HRAM).
// This is the module the CPU connects to -- a single
// CPU-facing bus (addr/din/dout/we) that behaves correctly across the
// whole Game Boy memory map.
//
// VRAM, OAM, external cart RAM, and the generic I/O region aren't built
// yet (VRAM/OAM belong to the video/ work, I/O spans several not-yet-
// built peripherals like the joypad). Rather than stub them out here,
// their ports pass straight through to the outside world unchanged --
// whoever wires up vram.v/oam.v/joypad.v later connects directly to
// these same ports, and this file doesn't need to change when they do.
//
// ROM_FILE is a parameter (not hardcoded) specifically so a testbench
// can point it at a known synthetic pattern instead of a real cartridge
// dump -- see sim/memory_top_tb.v.
//=============================================================================
module memory_top #(
    parameter ROM_FILE = "tetris.hex"
) (
    input  wire        clk,
    input  wire        rst,

    // ---- CPU-facing bus ---------------------------------------------------
    input  wire [15:0] addr,
    input  wire [7:0]  din,
    input  wire        we,
    output wire [7:0]  dout,

    // ---- Pass-through ports for regions not built yet --------------------
    output wire [12:0] vram_addr,
    output wire [7:0]  vram_din,
    output wire        vram_we,
    input  wire [7:0]  vram_dout,

    output wire [7:0]  oam_addr,
    output wire [7:0]  oam_din,
    output wire        oam_we,
    input  wire [7:0]  oam_dout,

    output wire [12:0] extram_addr,
    output wire [7:0]  extram_din,
    output wire        extram_we,
    input  wire [7:0]  extram_dout,

    output wire [6:0]  io_addr,
    output wire [7:0]  io_din,
    output wire        io_we,
    input  wire [7:0]  io_dout,

    // ---- Direct access for the CPU's interrupt logic (Person A) ----------
    output wire [7:0]  ie_reg,
    output wire [7:0]  if_reg
);

    // ---- ROM ---------------------------------------------------------------
    wire [14:0] rom_addr;
    wire [7:0]  rom_dout;
    rom #(.ROM_FILE(ROM_FILE), .ROM_SIZE(32768)) u_rom (
        .clk(clk), .addr(rom_addr), .dout(rom_dout)
    );

    // ---- WRAM ---------------------------------------------------------------
    wire [12:0] wram_addr;
    wire [7:0]  wram_din;
    wire        wram_we;
    wire [7:0]  wram_dout;
    wram #(.ADDR_WIDTH(13)) u_wram (
        .clk(clk), .addr(wram_addr), .din(wram_din), .we(wram_we), .dout(wram_dout)
    );

    // ---- HRAM ---------------------------------------------------------------
    wire [6:0] hram_addr;
    wire [7:0] hram_din;
    wire       hram_we;
    wire [7:0] hram_dout;
    wram #(.ADDR_WIDTH(7)) u_hram (
        .clk(clk), .addr(hram_addr), .din(hram_din), .we(hram_we), .dout(hram_dout)
    );

    // ---- Address decoder wires everything above + the pass-through ports -
    address_decoder u_decoder (
        .clk(clk), .rst(rst),
        .addr(addr), .din(din), .we(we), .dout(dout),

        .rom_addr(rom_addr), .rom_dout(rom_dout),

        .extram_addr(extram_addr), .extram_din(extram_din), .extram_we(extram_we), .extram_dout(extram_dout),

        .vram_addr(vram_addr), .vram_din(vram_din), .vram_we(vram_we), .vram_dout(vram_dout),

        .wram_addr(wram_addr), .wram_din(wram_din), .wram_we(wram_we), .wram_dout(wram_dout),

        .oam_addr(oam_addr), .oam_din(oam_din), .oam_we(oam_we), .oam_dout(oam_dout),

        .io_addr(io_addr), .io_din(io_din), .io_we(io_we), .io_dout(io_dout),

        .hram_addr(hram_addr), .hram_din(hram_din), .hram_we(hram_we), .hram_dout(hram_dout),

        .ie_reg(ie_reg), .if_reg(if_reg)
    );

endmodule