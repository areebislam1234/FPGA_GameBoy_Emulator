// cpu_control.v
// SM83 control FSM -- decodes and executes:
//   Block 2: ALU A,r8       Block 1: LD r8,r8' (incl. LD (HL),r8)
//   Block 0: LD r8,imm8 (incl. LD (HL),d8)
//   Plus unconditional control flow: JR imm8, JP imm16, JP (HL)
//
// Conditional jumps (JR cc,r8 / JP cc,a16) are NOT yet supported --
// deliberately scoped out, since unconditional jumps alone already
// introduce the first real break from "pc_reg only ever increments,"
// which every instruction before this assumed.
//
// JP imm16 needs a genuinely new capability: TWO separate immediate
// bytes (low, then high), read from two different memory cycles, before
// the jump target is even known. The existing pipeline only budgets one
// extra read cycle (MEM_READ) ahead of EXECUTE -- not enough for a
// second byte -- so this instruction alone detours through a new state,
// MEM_READ2. JR only needs one immediate byte (like LD r,imm8), so it
// fits the existing 4-cycle pattern. JP (HL) needs no immediate bytes
// at all -- it just redirects pc_reg to hl_pair directly in EXECUTE.
//
// HALT (opcode 0x76) is recognized and handled (the FSM parks in S_HALT
// and stays there), but since there's no interrupt controller yet,
// nothing currently wakes it back up -- a known gap for a later phase,
// not a bug.
//
// Memory interface is modeled as combinational read: mem_data_in is
// expected to reflect memory[mem_addr] within the same cycle. Writes are
// modeled as synchronous: mem_write_en/mem_write_data must be valid for
// the entire cycle a write should happen, sampled by external memory on
// the clock edge -- same convention as register_file's own write port.

module cpu_control (
    input  wire        clk,
    input  wire        rst,

    // Memory interface
    output reg  [15:0] mem_addr,
    input  wire [7:0]  mem_data_in,
    output wire        mem_write_en,
    output wire [7:0]  mem_write_data,

    // Observability -- useful for testbenches and later debugging
    output wire [15:0] pc,
    output wire [7:0]  ir,
    output reg         unimplemented,  // pulses 1 cycle on an unhandled opcode
    output wire        halted          // level signal: high while parked in HALT
);

    // Must match alu.v / register_file.v encodings
    localparam ALU_CP = 3'b111;
    localparam SEL_A  = 3'b111;

    localparam S_FETCH     = 3'b000,
               S_DECODE    = 3'b001,
               S_MEM_READ  = 3'b010,
               S_MEM_READ2 = 3'b011,
               S_EXECUTE   = 3'b100,
               S_MEM_WRITE = 3'b101,
               S_HALT      = 3'b110;

    reg [2:0]  state;
    reg [15:0] pc_reg;
    reg [7:0]  ir_reg;
    reg [7:0]  imm8_reg;  // latches the fetched immediate byte for LD (HL),d8's write
    reg [7:0]  jp_low_reg; // latches JP imm16's low byte between MEM_READ2 and EXECUTE

    assign pc     = pc_reg;
    assign ir     = ir_reg;
    assign halted = (state == S_HALT);

    // ---- Decode ----
    wire [2:0] alu_op       = ir_reg[5:3];  // Block 2: which ALU operation
    wire [2:0] dest_code    = ir_reg[5:3];  // Block 1: destination register
    wire [2:0] ld_imm8_dest = ir_reg[5:3];  // Block 0 LD r,imm8: destination register
    wire [2:0] reg_code     = ir_reg[2:0];  // source/operand register, shared field

    wire is_block0 = (ir_reg[7:6] == 2'b00);
    wire is_block1 = (ir_reg[7:6] == 2'b01);
    wire is_block2 = (ir_reg[7:6] == 2'b10);
    wire is_halt   = (ir_reg == 8'h76);

    // Fixed single-byte opcodes -- exact matches, same convention as HALT,
    // since none of these are defined by a bit-field pattern the way
    // blocks are.
    wire is_jr      = (ir_reg == 8'h18); // JR imm8 (unconditional)
    wire is_jp_imm16 = (ir_reg == 8'hC3); // JP imm16 (unconditional)
    wire is_jp_hl    = (ir_reg == 8'hE9); // JP (HL)

    wire is_mem_operand = (reg_code  == 3'b110); // source is (HL)
    wire is_dest_mem     = (dest_code == 3'b110); // Block 1 dest is (HL)

    wire is_ld_imm8          = is_block0 && (reg_code == 3'b110); // pattern 00 rrr 110
    wire is_ld_imm8_dest_mem = is_ld_imm8 && (ld_imm8_dest == 3'b110); // LD (HL),d8

    wire is_block1_regwrite = is_block1 && !is_halt && !is_dest_mem; // LD r,r' / LD r,(HL)
    wire is_block1_memwrite = is_block1 && !is_halt && is_dest_mem;  // LD (HL),r8
    wire is_supported_block1 = is_block1_regwrite || is_block1_memwrite;

    wire is_ld_imm8_regwrite = is_ld_imm8 && !is_ld_imm8_dest_mem; // LD r,d8
    wire is_ld_imm8_memwrite = is_ld_imm8_dest_mem;                 // LD (HL),d8
    wire is_supported_ld_imm8 = is_ld_imm8_regwrite || is_ld_imm8_memwrite;

    wire need_mem_read = (is_block2 || is_block1) && is_mem_operand && !is_halt;

    // Sign-extends JR's 8-bit relative offset to 16 bits so it can be
    // added directly to pc_reg. Concatenating 8 copies of the sign bit is
    // the standard Verilog idiom for this -- {8{bit}} repeats a single
    // bit 8 times, giving the correct two's-complement 16-bit value.
    wire [15:0] jr_offset_ext = {{8{mem_data_in[7]}}, mem_data_in};

    wire [7:0]  reg_a_data;
    wire [7:0]  reg_b_data;
    wire [3:0]  flags_out;
    wire [15:0] hl_pair;

    wire [7:0] resolved_b = is_mem_operand ? mem_data_in : reg_b_data;

    wire [7:0] alu_result;
    wire       alu_z, alu_n, alu_h, alu_c;

    wire do_writeback   = is_block2 && (alu_op != ALU_CP);
    wire do_flags_write = is_block2;

    wire [2:0] write_sel_mux  = is_ld_imm8_regwrite ? ld_imm8_dest :
                                is_block1_regwrite  ? dest_code   : SEL_A;
    wire [7:0] write_data_mux = is_ld_imm8_regwrite ? mem_data_in :
                                is_block1_regwrite   ? resolved_b : alu_result;

    wire in_execute  = (state == S_EXECUTE);
    wire rf_write_en = in_execute && (do_writeback || is_block1_regwrite || is_ld_imm8_regwrite);
    wire flags_write_en_w = in_execute && do_flags_write;

    wire mem_write_en_block1 = in_execute && is_block1_memwrite;
    wire mem_write_en_imm8   = (state == S_MEM_WRITE);
    assign mem_write_en   = mem_write_en_block1 || mem_write_en_imm8;
    assign mem_write_data = mem_write_en_imm8 ? imm8_reg : reg_b_data;

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
            imm8_reg      <= 8'h00;
            jp_low_reg    <= 8'h00;
            unimplemented <= 1'b0;
        end else begin
            unimplemented <= 1'b0; // default; only pulses in EXECUTE below

            case (state)
                S_FETCH: begin
                    mem_addr <= pc_reg;
                    state    <= S_DECODE;
                end

                S_DECODE: begin
                    ir_reg <= mem_data_in;
                    pc_reg <= pc_reg + 1'b1;
                    state  <= S_MEM_READ;
                end

                S_MEM_READ: begin
                    if (need_mem_read) begin
                        mem_addr <= hl_pair;
                    end else if (is_ld_imm8) begin
                        mem_addr <= pc_reg;
                    end else if (is_block1_memwrite) begin
                        mem_addr <= hl_pair;
                    end else if (is_jr) begin
                        mem_addr <= pc_reg;       // fetch the relative offset byte
                    end else if (is_jp_imm16) begin
                        mem_addr <= pc_reg;       // fetch the low byte of the target address
                    end
                    // is_jp_hl needs no redirect at all -- no immediate bytes involved
                    state <= is_jp_imm16 ? S_MEM_READ2 : S_EXECUTE;
                end

                S_MEM_READ2: begin
                    // Only ever entered for JP imm16. mem_data_in now holds
                    // the low byte (mem_addr was set to pc_reg last cycle).
                    jp_low_reg <= mem_data_in;
                    mem_addr   <= pc_reg + 1'b1; // point at the high byte next
                    pc_reg     <= pc_reg + 1'b1; // consume the low byte
                    state      <= S_EXECUTE;
                end

                S_EXECUTE: begin
                    if (is_ld_imm8) begin
                        pc_reg <= pc_reg + 1'b1;
                    end else if (is_jr) begin
                        // Consume the offset byte AND apply the signed jump
                        // in one step: pc_reg right now already points just
                        // past the offset byte's address, matching real
                        // hardware's "relative to the byte after this
                        // instruction" semantics.
                        pc_reg <= pc_reg + 1'b1 + jr_offset_ext;
                    end else if (is_jp_imm16) begin
                        // High byte just arrived on mem_data_in; low byte
                        // was latched last cycle. Assemble the full target.
                        pc_reg <= {mem_data_in, jp_low_reg};
                    end else if (is_jp_hl) begin
                        pc_reg <= hl_pair;
                    end

                    if (is_ld_imm8_memwrite) begin
                        imm8_reg <= mem_data_in;
                        mem_addr <= hl_pair;
                    end

                    unimplemented <= !is_block2 && !is_supported_block1 &&
                                      !is_halt && !is_supported_ld_imm8 &&
                                      !is_jr && !is_jp_imm16 && !is_jp_hl;

                    if (is_ld_imm8_memwrite) begin
                        state <= S_MEM_WRITE;
                    end else begin
                        state <= is_halt ? S_HALT : S_FETCH;
                    end
                end

                S_MEM_WRITE: begin
                    state <= S_FETCH;
                end

                S_HALT: begin
                    state <= S_HALT;
                end

                default: state <= S_FETCH;
            endcase
        end
    end

endmodule
