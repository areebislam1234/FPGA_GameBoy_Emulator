`timescale 1ns / 1ps
//=============================================================================
// bg_pixel_calc.v
// 
//
// All the Game-Boy-specific addressing/decoding math for background tile
// rendering, as pure combinational logic -- deliberately no clock, no
// state, at all. This is a fetch-timing-agnostic "given a background
// pixel position, what VRAM addresses and bit math produce its color"
// calculator; the sequential piece that actually walks through VRAM reads
// across clock cycles (prefetching one tile ahead so it's ready in time,
// the way real PPU hardware pipelines this) is a separate module that
// wraps around this one. Splitting it this way means the trickier
// prefetch timing gets its own focused testbench later, and this piece
// -- despite doing all the real GB-specific work -- has zero clock-edge
// timing to get wrong in the first place.
//
// Three combinational stages, used in sequence by the caller (each stage
// needs the previous VRAM read's result fed back in before its output is
// meaningful):
//
//   1. map_addr        <- gb_x, gb_y, scx, scy
//      Caller reads VRAM at map_addr, gets back a tile index.
//   2. data_addr0/1     <- tile_index (fed back), gb_y, scy
//      Caller reads VRAM at both addresses, gets back the tile's two
//      bit-plane bytes for the correct row.
//   3. color_index      <- tile_byte0/1 (fed back), gb_x, scx
//      The final 2-bit Game Boy color (0-3) for this exact pixel.
//
// Addressing choices fixed for this pass (documented, not accidental):
//   - Tile map base $9800 (LCDC bit 3 = 0 case). The $9C00 alternate map
//     is a one-constant change (map_addr's 13'h1800 offset) whenever it's
//     needed -- not built now since nothing in this project uses it yet.
//   - Tile data addressing mode $8000 / unsigned tile index (LCDC bit 4 =
//     1 case). The signed $8800 mode needs different math (tile index
//     interpreted as signed, base $9000) and is deliberately deferred
//     for the same reason.
//
// All VRAM addresses here are VRAM-LOCAL (0x0000-0x1FFF, i.e. already
// offset from the real $8000 base) -- the same convention address_decoder.v
// and vram will use, so these plug straight into a real vram_addr port
// with no extra translation.
//=============================================================================
module bg_pixel_calc (
    // ---- Stage 1: tile map lookup --------------------------------------
    input  wire [7:0]  gb_x,   // 0-159, Game Boy viewport X (native res)
    input  wire [7:0]  gb_y,   // 0-143, Game Boy viewport Y (native res)
    input  wire [7:0]  scx,    // background scroll X (SCX, FF43)
    input  wire [7:0]  scy,    // background scroll Y (SCY, FF42)
    output wire [12:0] map_addr,

    // ---- Stage 2: tile data lookup, needs tile_index fed back ----------
    input  wire [7:0]  tile_index,
    output wire [12:0] data_addr0,  // low bit-plane byte
    output wire [12:0] data_addr1,  // high bit-plane byte (== data_addr0 + 1)

    // ---- Stage 3: final pixel color, needs both data bytes fed back ----
    input  wire [7:0]  tile_byte0,
    input  wire [7:0]  tile_byte1,
    output wire [1:0]  color_index,

    // ---- Exposed for the sequential fetcher to track tile boundaries ---
    output wire [2:0]  col_within_tile  // 0-7; wraps 7->0 at each tile edge
);

    // ---- Scrolled background-space coordinates, with 8-bit wraparound --
    // gb_x/scx and gb_y/scy are both 8-bit; the sum naturally wraps mod
    // 256 when truncated back to 8 bits -- exactly the wraparound real
    // hardware wants for a 256x256 background map, with no explicit
    // modulo needed.
    wire [7:0] scrolled_x = gb_x + scx;
    wire [7:0] scrolled_y = gb_y + scy;

    // ---- Stage 1: which of the 32x32 map tiles, and which VRAM byte ----
    wire [4:0] tile_col = scrolled_x[7:3]; // divide by 8 -> 0-31
    wire [4:0] tile_row = scrolled_y[7:3]; // divide by 8 -> 0-31
    // {tile_row, tile_col} is exactly tile_row*32 + tile_col since
    // tile_col is always < 32 -- concatenation instead of a multiply.
    assign map_addr = 13'h1800 + {5'b0, tile_row, tile_col};

    // ---- Stage 2: tile data address for the correct row ------------------
    wire [2:0] row_within_tile = scrolled_y[2:0];
    assign data_addr0 = (tile_index << 4) + (row_within_tile << 1);
    assign data_addr1 = data_addr0 + 13'd1;

    // ---- Stage 3: extract this pixel's 2-bit color from the two bytes --
    // Bit 7 of each byte is the LEFTMOST pixel (MSB-first encoding);
    // low bit-plane in tile_byte0, high bit-plane in tile_byte1.
    assign col_within_tile = scrolled_x[2:0];
    assign color_index = {tile_byte1[3'd7 - col_within_tile], tile_byte0[3'd7 - col_within_tile]};

endmodule