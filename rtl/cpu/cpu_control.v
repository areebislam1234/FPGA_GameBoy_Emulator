// cpu_control.v
// SM83 control FSM -- decodes and executes:
//   Block 2: ALU A,r8       Block 1: LD r8,r8' (incl. LD (HL),r8)
//   Block 0: LD r8,imm8 (incl. LD (HL),d8)
//
// With the memory-write port added, Block 1 and Block 0's LD-immediate
// are now FULLY supported -- no more carved-out cases in either.
//
// LD (HL),d8 needs two separate memory operations: read the immediate
// byte (from pc_reg), then write it somewhere else entirely (hl_pair).
// The existing 4-state pipeline only budgets one extra memory cycle
// (MEM_READ) ahead of EXECUTE, which isn't enough room for both a read
// and a write to different addresses -- so this case alone detours
// through a 5th state, MEM_WRITE, after EXECUTE. LD (HL),r8 doesn't need
// this, since it has no immediate byte to fetch -- the value to write
// (a register's current contents) is available combinationally with no
// extra latency, so it completes within the normal 4 cycles.
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
               S_EXECUTE   = 3'b011,
               S_MEM_WRITE = 3'b100,
               S_HALT      = 3'b101;

    reg [2:0]  state;
    reg [15:0] pc_reg;
    reg [7:0]  ir_reg;
    reg [7:0]  imm8_reg; // latches the fetched immediate byte for LD (HL),d8's write

    assign pc     = pc_reg;
    assign ir     = ir_reg;
    assign halted = (state == S_HALT);

    // ---- Decode ----
    // Same 3-bit fields, reused with different meanings depending on which
    // block the top 2 bits point to -- a recurring SM83 pattern.
    wire [2:0] alu_op       = ir_reg[5:3];  // Block 2: which ALU operation
    wire [2:0] dest_code    = ir_reg[5:3];  // Block 1: destination register
    wire [2:0] ld_imm8_dest = ir_reg[5:3];  // Block 0 LD r,imm8: destination register
    wire [2:0] reg_code     = ir_reg[2:0];  // source/operand register, shared field

    wire is_block0 = (ir_reg[7:6] == 2'b00);
    wire is_block1 = (ir_reg[7:6] == 2'b01);
    wire is_block2 = (ir_reg[7:6] == 2'b10);
    wire is_halt   = (ir_reg == 8'h76);

    wire is_mem_operand = (reg_code  == 3'b110); // source is (HL)
    wire is_dest_mem     = (dest_code == 3'b110); // Block 1 dest is (HL)

    wire is_ld_imm8          = is_block0 && (reg_code == 3'b110); // pattern 00 rrr 110
    wire is_ld_imm8_dest_mem = is_ld_imm8 && (ld_imm8_dest == 3'b110); // LD (HL),d8

    // Block 1, split by which kind of write it needs.
    wire is_block1_regwrite = is_block1 && !is_halt && !is_dest_mem; // LD r,r' / LD r,(HL)
    wire is_block1_memwrite = is_block1 && !is_halt && is_dest_mem;  // LD (HL),r8 -- NEW
    wire is_supported_block1 = is_block1_regwrite || is_block1_memwrite; // all of Block 1 now

    // Block 0 LD imm8, split the same way.
    wire is_ld_imm8_regwrite = is_ld_imm8 && !is_ld_imm8_dest_mem; // LD r,d8
    wire is_ld_imm8_memwrite = is_ld_imm8_dest_mem;                 // LD (HL),d8 -- NEW
    wire is_supported_ld_imm8 = is_ld_imm8_regwrite || is_ld_imm8_memwrite; // all of it now

    wire need_mem_read = (is_block2 || is_block1) && is_mem_operand && !is_halt;

    wire [7:0]  reg_a_data;
    wire [7:0]  reg_b_data;
    wire [3:0]  flags_out;
    wire [15:0] hl_pair;

    // When the operand is (HL), mem_addr was pointed at hl_pair during
    // MEM_READ, so mem_data_in already reflects that byte here -- no
    // extra latching register needed for reads. LD (HL),d8's write value
    // DOES need latching (imm8_reg above), since mem_addr moves on to
    // hl_pair before the write actually happens.
    wire [7:0] resolved_b = is_mem_operand ? mem_data_in : reg_b_data;

    wire [7:0] alu_result;
    wire       alu_z, alu_n, alu_h, alu_c;

    wire do_writeback   = is_block2 && (alu_op != ALU_CP); // CP sets flags only
    wire do_flags_write = is_block2;                        // LD never touches flags

    // Register-file write path -- only the REGWRITE variants apply here;
    // the MEMWRITE variants (LD (HL),r8 / LD (HL),d8) write memory instead,
    // handled separately below via mem_write_en/mem_write_data.
    wire [2:0] write_sel_mux  = is_ld_imm8_regwrite ? ld_imm8_dest :
                                is_block1_regwrite  ? dest_code   : SEL_A;
    wire [7:0] write_data_mux = is_ld_imm8_regwrite ? mem_data_in :
                                is_block1_regwrite   ? resolved_b : alu_result;

    // MUST be combinational, not registered -- register_file's own
    // posedge-triggered write needs write_en high for the entire cycle
    // the write data is valid, not a pulse that arrives one edge late.
    wire in_execute  = (state == S_EXECUTE);
    wire rf_write_en = in_execute && (do_writeback || is_block1_regwrite || is_ld_imm8_regwrite);
    wire flags_write_en_w = in_execute && do_flags_write;

    // Memory-write path. Two different reasons to write, active at two
    // different states:
    //   - LD (HL),r8 writes DURING S_EXECUTE (no immediate byte to wait on;
    //     the register value and hl_pair address are both already valid).
    //   - LD (HL),d8 writes DURING S_MEM_WRITE (needs the extra cycle,
    //     since mem_addr was busy fetching the immediate byte during the
    //     cycle before EXECUTE).
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
                    // ir_reg is valid now, so all decode wires above are
                    // trustworthy here.
                    if (need_mem_read) begin
                        mem_addr <= hl_pair;       // Block 1/2 (HL) source read
                    end else if (is_ld_imm8) begin
                        mem_addr <= pc_reg;         // fetch the immediate byte
                    end else if (is_block1_memwrite) begin
                        mem_addr <= hl_pair;        // prepare write address one cycle ahead
                    end
                    state <= S_EXECUTE;
                end

                S_EXECUTE: begin
                    if (is_ld_imm8) begin
                        // Consume the immediate byte from the instruction
                        // stream either way -- pc must skip past it.
                        pc_reg <= pc_reg + 1'b1;
                    end

                    if (is_ld_imm8_memwrite) begin
                        // Capture the immediate byte NOW, while mem_addr
                        // still points at it (from MEM_READ) -- next cycle
                        // mem_addr moves to hl_pair for the actual write.
                        imm8_reg <= mem_data_in;
                        mem_addr <= hl_pair;
                    end

                    unimplemented <= !is_block2 && !is_supported_block1 &&
                                      !is_halt && !is_supported_ld_imm8;

                    if (is_ld_imm8_memwrite) begin
                        state <= S_MEM_WRITE;
                    end else begin
                        state <= is_halt ? S_HALT : S_FETCH;
                    end
                end

                S_MEM_WRITE: begin
                    // mem_addr already = hl_pair (set last cycle in EXECUTE);
                    // mem_write_en/mem_write_data are combinational and
                    // already valid throughout this whole state.
                    state <= S_FETCH;
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
