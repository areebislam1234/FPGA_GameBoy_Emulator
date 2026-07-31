`timescale 1ns / 1ps
//=============================================================================
// tile_fetcher.v
// 
//
// Sequential prefetch pipeline that walks bg_pixel_calc.v through actual
// VRAM reads, one tile ahead of what's being displayed -- the piece
// bg_pixel_calc.v was deliberately built without, so this is the only
// place clock-edge timing for background rendering has to be gotten
// right.
//
// Deliberately decoupled from VGA timing: this module advances on its own
// `gb_pixel_tick` input, meaning "display the next GB-native pixel" (0-159
// x, 0-143 y, wrapping). How that relates to vga_controller.v's 4x-faster
// pixel_tick (each GB pixel becomes a 4x4 block of VGA pixels) is a
// separate concern for whatever integrates this later (ppu.v) -- this
// module doesn't need to know about VGA resolution or scaling at all,
// which keeps its own verification self-contained.
//
// ---------------------------------------------------------------------------
// WHY EACH VRAM READ TAKES 3 CYCLES, NOT 1:
//
// VRAM (synchronous-read BRAM, same contract as every other memory in this
// project) needs: address presented cycle N -> data valid cycle N+1. So
// reading requires an ISSUE cycle (drive the address) and a CAPTURE cycle
// (read the now-valid data) -- 2 cycles minimum per read.
//
// But the tile-data reads need the tile index from the MAP read first
// (bg_pixel_calc.v's data_addr0/1 outputs depend on its tile_index input),
// and that dependency has its own hazard: if the map read's result is
// captured into a register with `<=` and the data address is computed
// from that same register in the SAME cycle, the combinational data_addr0
// would still reflect the OLD (pre-update) tile_index -- non-blocking
// assignments don't become visible until the NEXT cycle. So there's a
// third, WAIT cycle between capturing the tile index and trusting the
// data address computed from it. Net result: ISSUE -> WAIT -> CAPTURE,
// 3 cycles per read for the map byte and for data byte 0; data byte 1
// reuses the already-settled tile index so it only needs ISSUE -> CAPTURE,
// 2 more cycles. 8 states total (F_MAP through F_D1_CAP).
//
// ---------------------------------------------------------------------------
// TIMING PRECONDITION: gb_pixel_tick must not fire more than once per 10
// clk cycles. This number is verified, not estimated -- an earlier version
// of this comment (and of sim/tile_fetcher_tb.v's boundary test) claimed 8
// cycles based on just counting the fetch sub-sequence's 8 states, which
// left out two real sources of latency: one cycle between a trigger firing
// and the (latched) fetch request actually being visible to consume, and
// one cycle between the fetch's last state writing its result and that
// result + the FSM's return to idle actually being visible to the next
// trigger. Tracing it through explicitly, cycle by cycle: trigger fires at
// cycle T -> request visible/consumed at T+1 -> the 8 fetch states run
// T+2 through T+9 -> result data and FSM-idle both become visible at T+10.
// A trigger arriving any earlier than T+10 would find either stale data
// (if it swaps in a fetch that hasn't finished) or a busy FSM. The 8-cycle
// version of this test genuinely failed in simulation before being
// corrected -- worth keeping that history visible here, since "count the
// states" is a natural way to get this number wrong by exactly the amount
// this project got it wrong by.
//
// Two-part fix that got the design to this point, both real bugs, not
// testbench artifacts:
//   1. The fetch request is a LATCHED pending flag (pending_fetch), not a
//      one-cycle pulse. An earlier version used a pulse, checked only by
//      the F_IDLE state -- if a new trigger arrived while a previous fetch
//      was still mid-sequence (not in F_IDLE), the pulse was silently
//      dropped and that fetch simply never happened. Latching means a
//      request that arrives "too early" just waits until F_IDLE is ready
//      to consume it, instead of vanishing.
//   2. Even with latching, a trigger arriving before T+10 finds a fetch
//      that hasn't produced valid data yet -- latching prevents a DROPPED
//      request, but the true minimum spacing (10 cycles) still has to be
//      respected for the SWAPPED-IN data itself to be correct, since the
//      swap doesn't wait for the fetch to finish -- it fires unconditionally
//      whenever entering_new_tile fires.
//
// In the real integration (via ppu.v), gb_pixel_tick is derived from
// vga_controller's pixel_tick with 4x horizontal scaling, giving ~16 clk
// cycles between GB pixel ticks at minimum -- comfortable margin over the
// 10 actually required.
//
// ---------------------------------------------------------------------------
// PREFETCH ARCHITECTURE:
//
// cur_byte0/1  -- the tile currently being displayed (feeds a bg_pixel_calc
//                 instance driven by the DISPLAY position, disp_x/disp_y)
// next_byte0/1 -- the tile being prefetched for what comes after (feeds a
//                 SEPARATE bg_pixel_calc instance driven by the FETCH
//                 position, fetch_x/fetch_y, which stays exactly one tile
//                 ahead of the display position)
//
// Two bg_pixel_calc instances, not one, because fetch and display need
// DIFFERENT gb_x/gb_y simultaneously -- the fetcher is always working on
// the NEXT tile while the display side is still using the current one.
//
// On reset, two full fetches run back-to-back (into cur, then into next)
// before `primed` goes high -- gb_pixel_tick is ignored until then, since
// there's nothing valid to display yet.
//=============================================================================
module tile_fetcher (
    input  wire       clk,
    input  wire       rst,
    input  wire       gb_pixel_tick,  // pulse: advance to the next GB-native pixel

    input  wire [7:0] scx,
    input  wire [7:0] scy,

    output wire [12:0] vram_addr,     // read-only from this module
    input  wire [7:0]  vram_dout,

    output wire [1:0]  color_index,   // valid for the CURRENT disp_x/disp_y
    output reg         primed         // goes high once startup priming completes
);

    // ---- Display / fetch position tracking --------------------------------
    reg [7:0] disp_x, disp_y;   // currently displayed GB-native pixel
    reg [7:0] fetch_x, fetch_y; // tile currently being prefetched (one tile ahead)

    // ---- Tile data double-buffer --------------------------------------------
    reg [7:0] cur_byte0, cur_byte1;
    reg [7:0] next_byte0, next_byte1;

    // ---- bg_pixel_calc: FETCH-side instance (addressing only) -------------
    wire [12:0] fetch_map_addr, fetch_data_addr0, fetch_data_addr1;
    reg  [7:0]  fetch_tile_index;
    bg_pixel_calc u_fetch_calc (
        .gb_x(fetch_x), .gb_y(fetch_y), .scx(scx), .scy(scy),
        .map_addr(fetch_map_addr),
        .tile_index(fetch_tile_index),
        .data_addr0(fetch_data_addr0), .data_addr1(fetch_data_addr1),
        .tile_byte0(8'd0), .tile_byte1(8'd0),   // unused on this instance
        .color_index(), .col_within_tile()       // unused on this instance
    );

    // ---- bg_pixel_calc: DISPLAY-side instance (color decode only) ---------
    wire [2:0] disp_col;
    bg_pixel_calc u_disp_calc (
        .gb_x(disp_x), .gb_y(disp_y), .scx(scx), .scy(scy),
        .map_addr(), .tile_index(8'd0), .data_addr0(), .data_addr1(), // unused
        .tile_byte0(cur_byte0), .tile_byte1(cur_byte1),
        .color_index(color_index),
        .col_within_tile(disp_col)
    );

    // ---- Fetch sub-sequence: one complete tile fetch, 8 cycles -----------
    localparam F_IDLE=0, F_MAP=1, F_MAP_WAIT=2, F_MAP_CAP=3,
               F_D0_SETUP=4, F_D0_WAIT=5, F_D0_CAP=6, F_D1_WAIT=7, F_D1_CAP=8;
    reg [3:0] fstate;
    reg [7:0] fetch_byte0_tmp;
    reg [12:0] vram_addr_r;
    assign vram_addr = vram_addr_r;

    reg fetch_done;      // pulse out to top-level control, fires when a fetch completes
    reg fetch_dest_next; // destination of the fetch CURRENTLY IN PROGRESS (copied from
                          // pending_dest_next when a pending request is consumed)

    reg pending_fetch;      // latched request: "a fetch needs to start"
    reg [7:0] pending_x, pending_y;
    reg pending_dest_next;  // destination for the PENDING request specifically

    // ---- Top-level control: priming sequence, then steady-state prefetch -
    localparam C_PRIME_CUR=0, C_PRIME_NEXT=1, C_STEADY=2;
    reg [1:0] cstate;

    // entering_new_tile fires on a genuine horizontal tile-column crossing
    // (disp_col==7) OR whenever the line itself ends (disp_x==159) -- the
    // second condition matters because when scx isn't a multiple of 8, the
    // line can end mid-tile (col_within_tile != 7 at disp_x=159), so a
    // fresh tile lookup is still needed for the next line even though no
    // "normal" column crossing occurred at that exact position.
    wire entering_new_tile = primed && gb_pixel_tick && ((disp_col == 3'd7) || (disp_x == 8'd159));

    // ---- Derive the fetch position fresh from where display is headed ----
    // Deliberately NOT tracked as an independent "+8 per crossing, wrap at
    // a fixed raw offset" counter -- an earlier version did that, assuming
    // every line touches exactly 20 tiles (true only when scx is a
    // multiple of 8). When scx isn't 8-aligned, the line starts mid-tile,
    // and a 160-pixel line can span 21 tiles instead of 20.
    wire [7:0] disp_x_next = (disp_x == 8'd159) ? 8'd0 : disp_x + 8'd1;
    wire [7:0] disp_y_next = (disp_x == 8'd159) ? ((disp_y == 8'd143) ? 8'd0 : disp_y + 8'd1) : disp_y;

    // Is display about to enter the LAST (possibly partial) tile of its
    // current line? If so, the tile needed one-tile-later isn't "+8 raw
    // pixels, wrapped mod 160" -- it's tile-column-zero of the FOLLOWING
    // row, because the line ends there. A raw-pixel-space wraparound
    // doesn't know that and silently computes the wrong tile whenever a
    // non-8-aligned scx makes the line span 21 tiles instead of 20 (this
    // was the actual bug an earlier version had).
    wire       entering_last_tile_of_line = (disp_x_next >= 8'd152);

    wire [7:0] derived_fetch_x = entering_last_tile_of_line ? 8'd0 : (disp_x_next + 8'd8);
    wire [7:0] derived_fetch_y = entering_last_tile_of_line
                                  ? ((disp_y_next == 8'd143) ? 8'd0 : disp_y_next + 8'd1)
                                  : disp_y_next;

    always @(posedge clk) begin
        if (rst) begin
            disp_x <= 8'd0; disp_y <= 8'd0;
            fetch_x <= 8'd0; fetch_y <= 8'd0;
            cur_byte0 <= 8'd0; cur_byte1 <= 8'd0;
            next_byte0 <= 8'd0; next_byte1 <= 8'd0;
            primed <= 1'b0;
            fstate <= F_IDLE;
            cstate <= C_PRIME_CUR;
            pending_fetch <= 1'b1;      // kick off the very first fetch immediately
            pending_x <= 8'd0; pending_y <= 8'd0;
            pending_dest_next <= 1'b0; // first fetch (tile at 0,0) goes into cur
            fetch_dest_next <= 1'b0;
            fetch_done <= 1'b0;
            fetch_tile_index <= 8'd0;
            fetch_byte0_tmp <= 8'd0;
            vram_addr_r <= 13'd0;
        end else begin

            fetch_done <= 1'b0; // default; pulses for exactly one cycle in F_D1_CAP

            // ---- Display-side position advance (ignored until primed) ----
            if (primed && gb_pixel_tick) begin
                if (disp_x == 8'd159) begin
                    disp_x <= 8'd0;
                    disp_y <= (disp_y == 8'd143) ? 8'd0 : disp_y + 8'd1;
                end else begin
                    disp_x <= disp_x + 8'd1;
                end
            end

            // ---- Swap buffers exactly when crossing into a new tile -------
            if (entering_new_tile) begin
                cur_byte0 <= next_byte0;
                cur_byte1 <= next_byte1;
            end

            // ---- Fetch sub-sequence -----------------------------------------
            case (fstate)
                F_IDLE: begin
                    if (pending_fetch) begin
                        fetch_x <= pending_x;
                        fetch_y <= pending_y;
                        fetch_dest_next <= pending_dest_next;
                        pending_fetch <= 1'b0;
                        fstate <= F_MAP;
                    end
                end
                F_MAP: begin
                    vram_addr_r <= fetch_map_addr;
                    fstate <= F_MAP_WAIT;
                end
                F_MAP_WAIT: begin
                    fstate <= F_MAP_CAP; // VRAM capturing the address now; nothing to read yet
                end
                F_MAP_CAP: begin
                    fetch_tile_index <= vram_dout; // valid now
                    fstate <= F_D0_SETUP;
                end
                F_D0_SETUP: begin
                    // fetch_tile_index settled last cycle -- fetch_data_addr0 is
                    // now combinationally correct
                    vram_addr_r <= fetch_data_addr0;
                    fstate <= F_D0_WAIT;
                end
                F_D0_WAIT: begin
                    fstate <= F_D0_CAP;
                end
                F_D0_CAP: begin
                    fetch_byte0_tmp <= vram_dout; // valid now
                    // data_addr1 depends only on the already-settled tile
                    // index, so it's safe to use combinationally here with
                    // no extra settle cycle
                    vram_addr_r <= fetch_data_addr1;
                    fstate <= F_D1_WAIT;
                end
                F_D1_WAIT: begin
                    fstate <= F_D1_CAP;
                end
                F_D1_CAP: begin
                    if (fetch_dest_next) begin
                        next_byte0 <= fetch_byte0_tmp;
                        next_byte1 <= vram_dout;
                    end else begin
                        cur_byte0 <= fetch_byte0_tmp;
                        cur_byte1 <= vram_dout;
                    end
                    fetch_done <= 1'b1;
                    fstate <= F_IDLE;
                end
                default: fstate <= F_IDLE;
            endcase

            // ---- Top-level control --------------------------------------------
            case (cstate)
                C_PRIME_CUR: begin
                    if (fetch_done) begin
                        pending_x <= 8'd8; // one tile ahead of the first (0,0)
                        pending_y <= 8'd0;
                        pending_dest_next <= 1'b1;
                        pending_fetch <= 1'b1;
                        cstate <= C_PRIME_NEXT;
                    end
                end
                C_PRIME_NEXT: begin
                    if (fetch_done) begin
                        primed <= 1'b1;
                        cstate <= C_STEADY;
                    end
                end
                C_STEADY: begin
                    if (entering_new_tile) begin
                        // Latched, not pulsed: if a previous fetch is still
                        // in progress (fstate not yet back to F_IDLE) when
                        // this fires, the request waits here instead of
                        // being silently dropped -- F_IDLE's own check
                        // above picks it up as soon as it's ready, however
                        // many cycles that ends up being.
                        pending_x <= derived_fetch_x;
                        pending_y <= derived_fetch_y;
                        pending_dest_next <= 1'b1;
                        pending_fetch <= 1'b1;
                    end
                end
                default: cstate <= C_STEADY;
            endcase

        end
    end

endmodule