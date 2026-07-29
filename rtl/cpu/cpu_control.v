// cpu_control.v
// SM83 control FSM -- decodes and executes:
//   Block 2: ALU A,r8       Block 1: LD r8,r8' (incl. LD (HL),r8)
//   Block 0: LD r8,imm8 (incl. LD (HL),d8), LD r16,imm16, NOP
//   Plus unconditional control flow: JR imm8, JP imm16, JP (HL)
//
// LD r16,imm16 reuses the same "fetch low byte, then high byte" machinery
// JP imm16 already needed (MEM_READ2), but differs in what happens once
// both bytes are known:
//   - LD SP,imm16 writes register_file's dedicated 16-bit SP port in one
//     shot, same cycle as everything else in EXECUTE.
//   - LD BC/DE/HL,imm16 is genuinely different: register_file only
//     exposes ONE 8-bit write port, so loading a pair means TWO separate
//     writes (high byte, then low byte) across two cycles. That's why
//     this specific case detours through a new state, S_REG_WRITE_LOW,
//     after EXECUTE -- the same "add a state when one cycle truly isn't
//     enough" pattern used earlier for LD (HL),d8 and JP imm16.
//
// NOP (0x00) is now explicitly recognized rather than falling through to
// `unimplemented` -- it was incorrectly flagged before since nothing
// excluded it; a true no-op should read as "handled, does nothing," not
// "opcode this FSM doesn't understand."
//
// Conditional jumps are still NOT supported -- deliberately scoped out.
//
// Memory interface: combinational read, synchronous write (see prior
// history for the full rationale). HALT parks the FSM with no wake-up
// yet (no interrupt controller).

module cpu_control (
    input  wire        clk,
    input  wire        rst,

    // Memory interface
    output reg  [15:0] mem_addr,
    input  wire [7:0]  mem_data_in,
    output wire        mem_write_en,
    output wire [7:0]  mem_write_data,

    // Observability
    output wire [15:0] pc,
    output wire [7:0]  ir,
    output reg         unimplemented,
    output wire        halted
);

    localparam ALU_CP = 3'b111;
    localparam SEL_A  = 3'b111;

    localparam S_FETCH        = 3'b000,
               S_DECODE       = 3'b001,
               S_MEM_READ     = 3'b010,
               S_MEM_READ2    = 3'b011,
               S_EXECUTE      = 3'b100,
               S_MEM_WRITE    = 3'b101,
               S_HALT         = 3'b110,
               S_REG_WRITE_LOW = 3'b111;

    reg [2:0]  state;
    reg [15:0] pc_reg;
    reg [7:0]  ir_reg;
    reg [7:0]  imm8_reg;
    reg [7:0]  jp_low_reg; // latches the low byte for JP imm16 AND LD r16,imm16

    assign pc     = pc_reg;
    assign ir     = ir_reg;
    assign halted = (state == S_HALT);

    // ---- Decode ----
    wire [2:0] alu_op       = ir_reg[5:3];
    wire [2:0] dest_code    = ir_reg[5:3];
    wire [2:0] ld_imm8_dest = ir_reg[5:3];
    wire [2:0] reg_code     = ir_reg[2:0];
    wire [1:0] r16_sel      = ir_reg[5:4]; // Block 0 LD r16,imm16: which pair (BC/DE/HL/SP)

    wire is_block0 = (ir_reg[7:6] == 2'b00);
    wire is_block1 = (ir_reg[7:6] == 2'b01);
    wire is_block2 = (ir_reg[7:6] == 2'b10);
    wire is_halt   = (ir_reg == 8'h76);
    wire is_nop    = (ir_reg == 8'h00);

    wire is_jr       = (ir_reg == 8'h18);
    wire is_jp_imm16 = (ir_reg == 8'hC3);
    wire is_jp_hl    = (ir_reg == 8'hE9);

    wire is_mem_operand = (reg_code  == 3'b110);
    wire is_dest_mem     = (dest_code == 3'b110);

    wire is_ld_imm8          = is_block0 && (reg_code == 3'b110);
    wire is_ld_imm8_dest_mem = is_ld_imm8 && (ld_imm8_dest == 3'b110);

    // LD r16,imm16 pattern: 00 dd 0001
    wire is_ld_r16_imm16      = is_block0 && (ir_reg[3:0] == 4'b0001);
    wire is_ld_r16_imm16_sp   = is_ld_r16_imm16 && (r16_sel == 2'b11); // SP
    wire is_ld_r16_imm16_pair = is_ld_r16_imm16 && (r16_sel != 2'b11); // BC/DE/HL

    // dd concatenated with a trailing 0/1 bit gives exactly the matching
    // 8-bit register codes -- e.g. dd=00 (BC): {00,0}=000=B, {00,1}=001=C.
    // Same encoding register_file.v already uses, no coincidence.
    wire [2:0] reg16_hi_sel = {r16_sel, 1'b0};
    wire [2:0] reg16_lo_sel = {r16_sel, 1'b1};

    wire is_block1_regwrite = is_block1 && !is_halt && !is_dest_mem;
    wire is_block1_memwrite = is_block1 && !is_halt && is_dest_mem;
    wire is_supported_block1 = is_block1_regwrite || is_block1_memwrite;

    wire is_ld_imm8_regwrite = is_ld_imm8 && !is_ld_imm8_dest_mem;
    wire is_ld_imm8_memwrite = is_ld_imm8_dest_mem;
    wire is_supported_ld_imm8 = is_ld_imm8_regwrite || is_ld_imm8_memwrite;

    wire need_mem_read = (is_block2 || is_block1) && is_mem_operand && !is_halt;

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

    wire in_execute       = (state == S_EXECUTE);
    wire in_reg_write_low = (state == S_REG_WRITE_LOW);

    // Register-file write port mux -- priority-ordered since these states
    // are mutually exclusive (never true at the same time), but written
    // this way so the priority is explicit rather than assumed.
    wire [2:0] write_sel_mux =
        in_reg_write_low       ? reg16_lo_sel  :
        is_ld_r16_imm16_pair   ? reg16_hi_sel  :
        is_ld_imm8_regwrite    ? ld_imm8_dest  :
        is_block1_regwrite     ? dest_code     : SEL_A;

    wire [7:0] write_data_mux =
        in_reg_write_low       ? jp_low_reg  :
        is_ld_r16_imm16_pair   ? mem_data_in :  // high byte, valid during EXECUTE
        is_ld_imm8_regwrite    ? mem_data_in :
        is_block1_regwrite     ? resolved_b  : alu_result;

    wire rf_write_en =
        (in_execute && (do_writeback || is_block1_regwrite ||
                        is_ld_imm8_regwrite || is_ld_r16_imm16_pair))
        || in_reg_write_low;

    wire flags_write_en_w = in_execute && do_flags_write;

    wire sp_write_en_w  = in_execute && is_ld_r16_imm16_sp;
    wire [15:0] sp_write_data_w = {mem_data_in, jp_low_reg};

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
        .sp_write_en(sp_write_en_w), .sp_write_data(sp_write_data_w), .sp(),
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
            unimplemented <= 1'b0;

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
                        mem_addr <= pc_reg;
                    end else if (is_jp_imm16 || is_ld_r16_imm16) begin
                        mem_addr <= pc_reg; // fetch the low byte
                    end
                    state <= (is_jp_imm16 || is_ld_r16_imm16) ? S_MEM_READ2 : S_EXECUTE;
                end

                S_MEM_READ2: begin
                    // Shared by JP imm16 and LD r16,imm16 -- both need a
                    // low byte latched and the high byte's address queued up.
                    jp_low_reg <= mem_data_in;
                    mem_addr   <= pc_reg + 1'b1;
                    pc_reg     <= pc_reg + 1'b1;
                    state      <= S_EXECUTE;
                end

                S_EXECUTE: begin
                    if (is_ld_imm8) begin
                        pc_reg <= pc_reg + 1'b1;
                    end else if (is_jr) begin
                        pc_reg <= pc_reg + 1'b1 + jr_offset_ext;
                    end else if (is_jp_imm16) begin
                        pc_reg <= {mem_data_in, jp_low_reg};
                    end else if (is_jp_hl) begin
                        pc_reg <= hl_pair;
                    end else if (is_ld_r16_imm16) begin
                        // Consume the high byte -- applies to both the SP
                        // and pair variants, regardless of which write path
                        // they take next.
                        pc_reg <= pc_reg + 1'b1;
                    end

                    if (is_ld_imm8_memwrite) begin
                        imm8_reg <= mem_data_in;
                        mem_addr <= hl_pair;
                    end

                    unimplemented <= !is_block2 && !is_supported_block1 &&
                                      !is_halt && !is_supported_ld_imm8 &&
                                      !is_jr && !is_jp_imm16 && !is_jp_hl &&
                                      !is_ld_r16_imm16 && !is_nop;

                    if (is_ld_imm8_memwrite) begin
                        state <= S_MEM_WRITE;
                    end else if (is_ld_r16_imm16_pair) begin
                        state <= S_REG_WRITE_LOW;
                    end else begin
                        state <= is_halt ? S_HALT : S_FETCH;
                    end
                end

                S_MEM_WRITE: begin
                    state <= S_FETCH;
                end

                S_REG_WRITE_LOW: begin
                    // The write itself happens combinationally (write_sel_mux/
                    // write_data_mux/rf_write_en, gated on in_reg_write_low) --
                    // this state just needs to exist for one cycle so that
                    // write fires, then return to normal fetching.
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
