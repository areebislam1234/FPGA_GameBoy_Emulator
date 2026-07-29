// cpu_control.v
// SM83 control FSM -- decodes and executes Block 2 (ALU A,r8) and
// Block 1 (LD r8,r8'), including the (HL) memory-operand cases.
//
// Deliberately NOT yet supported: LD (HL),r8 -- writing a register INTO
// memory needs a memory-write port this module doesn't have yet. That
// specific case is explicitly flagged via `unimplemented` rather than
// silently doing nothing. HALT (opcode 0x76) is recognized and handled
// (the FSM parks in S_HALT and stays there), but since there's no
// interrupt controller yet, nothing currently wakes it back up --
// that's a known gap for a later phase, not a bug.
//
// Memory interface is modeled as combinational read: mem_data_in is
// expected to reflect memory[mem_addr] within the same cycle. Real block
// RAM is registered with a 1-cycle read latency -- FETCH and MEM_READ are
// deliberately separate states specifically so this FSM's timing survives
// that change later (Phase 7 integration) without restructuring.
//
// Every instruction takes a fixed 4 cycles (FETCH/DECODE/MEM_READ/EXECUTE),
// even ops that don't strictly need the MEM_READ cycle. Real SM83 timing
// varies per instruction -- this trades cycle-exact accuracy for a
// simpler first pass. Worth revisiting once Phase 3's timing-sensitive
// tests are in play.

module cpu_control (
    input  wire        clk,
    input  wire        rst,

    // Memory interface
    output reg  [15:0] mem_addr,
    input  wire [7:0]  mem_data_in,

    // Observability -- useful for testbenches and later debugging
    output wire [15:0] pc,
    output wire [7:0]  ir,
    output reg         unimplemented,  // pulses 1 cycle on an unhandled opcode
    output wire        halted          // level signal: high while parked in HALT
);

    // Must match alu.v / register_file.v encodings
    localparam ALU_CP = 3'b111;
    localparam SEL_A  = 3'b111;

    localparam S_FETCH    = 3'b000,
               S_DECODE   = 3'b001,
               S_MEM_READ = 3'b010,
               S_EXECUTE  = 3'b011,
               S_HALT     = 3'b100;

    reg [2:0]  state;
    reg [15:0] pc_reg;
    reg [7:0]  ir_reg;

    assign pc     = pc_reg;
    assign ir     = ir_reg;
    assign halted = (state == S_HALT);

    // ---- Decode ----
    wire [2:0] alu_op    = ir_reg[5:3];  // Block 2: which ALU operation
    wire [2:0] dest_code = ir_reg[5:3];  // Block 1: destination register (same bits, different meaning)
    wire [2:0] reg_code  = ir_reg[2:0];  // source/operand register, same field in both blocks

    wire is_block1 = (ir_reg[7:6] == 2'b01);
    wire is_block2 = (ir_reg[7:6] == 2'b10);
    wire is_halt   = (ir_reg == 8'h76);

    wire is_mem_operand = (reg_code  == 3'b110); // source is (HL)
    wire is_dest_mem     = (dest_code == 3'b110); // destination is (HL) -- LD (HL),r, not yet supported

    // Block 1 cases this FSM actually handles: LD r,r' and LD r,(HL).
    // Excludes HALT (its own state) and LD (HL),r (needs a memory write port).
    wire is_supported_block1 = is_block1 && !is_halt && !is_dest_mem;

    wire need_mem_read = (is_block2 || is_block1) && is_mem_operand && !is_halt;

    wire [7:0]  reg_a_data;
    wire [7:0]  reg_b_data;
    wire [3:0]  flags_out;
    wire [15:0] hl_pair;

    // When the operand is (HL), mem_addr was pointed at hl_pair during
    // MEM_READ, so mem_data_in already reflects that byte here -- no
    // extra latching register needed.
    wire [7:0] resolved_b = is_mem_operand ? mem_data_in : reg_b_data;

    wire [7:0] alu_result;
    wire       alu_z, alu_n, alu_h, alu_c;

    wire do_writeback   = is_block2 && (alu_op != ALU_CP); // CP sets flags only
    wire do_flags_write = is_block2;                        // LD never touches flags

    // write_sel/write_data depend on which block is executing:
    //   Block 2 always targets A with the ALU result.
    //   Block 1 targets whatever register the opcode names, with the
    //   plain source value (resolved_b already handles the (HL) case).
    wire [2:0] write_sel_mux  = is_block1 ? dest_code : SEL_A;
    wire [7:0] write_data_mux = is_block1 ? resolved_b : alu_result;

    // MUST be combinational, not registered -- register_file's own
    // posedge-triggered write needs write_en high for the entire cycle
    // the write data is valid (the whole S_EXECUTE state), not a pulse
    // that arrives one edge late relative to when it was computed.
    wire in_execute  = (state == S_EXECUTE);
    wire rf_write_en = in_execute && (do_writeback || is_supported_block1);
    wire flags_write_en_w = in_execute && do_flags_write;

    register_file rf (
        .clk(clk), .rst(rst),
        .write_en(rf_write_en), .write_sel(write_sel_mux), .write_data(write_data_mux),
        .read_sel_a(SEL_A), .read_sel_b(reg_code),
        .read_data_a(reg_a_data), .read_data_b(reg_b_data),
        .flags_write_en(flags_write_en_w),
        .flags_in({alu_z, alu_n, alu_h, alu_c}),
        .flags_out(flags_out),
        .sp_write_en(1'b0), .sp_write_data(16'h0000), .sp(),
        .bc(), .de(), .hl(hl_pair), .af()
    );

    alu al (
        .op(alu_op), .a(reg_a_data), .b(resolved_b), .carry_in(flags_out[0]),
        .result(alu_result), .flag_z(alu_z), .flag_n(alu_n),
        .flag_h(alu_h), .flag_c(alu_c)
    );

    always @(posedge clk) begin
        if (rst) begin
            state         <= S_FETCH;
            pc_reg        <= 16'h0000;
            ir_reg        <= 8'h00;
            mem_addr      <= 16'h0000;
            unimplemented <= 1'b0;
        end else begin
            unimplemented <= 1'b0; // default; only pulses in EXECUTE below

            case (state)
                S_FETCH: begin
                    mem_addr <= pc_reg;
                    state    <= S_DECODE;
                end

                S_DECODE: begin
                    ir_reg <= mem_data_in; // valid now -- mem_addr was set last cycle
                    pc_reg <= pc_reg + 1'b1;
                    state  <= S_MEM_READ;
                end

                S_MEM_READ: begin
                    // ir_reg is valid now (latched at the end of S_DECODE), so
                    // all the decode wires above are trustworthy this cycle.
                    if (need_mem_read) begin
                        mem_addr <= hl_pair;
                    end
                    state <= S_EXECUTE;
                end

                S_EXECUTE: begin
                    unimplemented <= !is_block2 && !is_supported_block1 && !is_halt;
                    state <= is_halt ? S_HALT : S_FETCH;
                end

                S_HALT: begin
                    // Parked here until a future phase adds interrupt-driven
                    // wake-up. Deliberately doesn't advance pc or re-fetch.
                    state <= S_HALT;
                end

                default: state <= S_FETCH;
            endcase
        end
    end

endmodule
