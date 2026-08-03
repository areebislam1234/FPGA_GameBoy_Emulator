// cpu_control.v
// SM83 control FSM. Supports:
//   Block 2: ALU A,r8       Block 1: LD r8,r8' (incl. LD (HL),r8)
//   Block 0: LD r8,imm8 (incl. LD (HL),d8), LD r16,imm16, NOP, INC/DEC r8, INC/DEC r16
//   Unconditional control flow: JR imm8, JP imm16, JP (HL), CALL imm16, RET
//   PUSH/POP BC,DE,HL
//
// TIMING MODEL CHANGE: this version assumes REGISTERED (synchronous) memory
// reads, matching real Block RAM (and this project's actual vram.v) --
// NOT the combinational-read assumption every earlier version used.
//
// On real hardware, presenting an address does not make data appear that
// same cycle. A registered-read RAM samples the address at a clock edge
// and only THEN produces data, valid starting the NEXT cycle. Every
// earlier version of this file assumed data was available one cycle
// after setting mem_addr; a genuinely registered memory needs a full
// EXTRA cycle of the address being stable before the data is valid.
//
// Rather than hand-inserting a uniquely-named wait state at every one of
// the many places this FSM reads memory, this version adds ONE shared
// S_MEM_WAIT state plus a next_state register: any transition that needs
// an extra settling cycle detours through S_MEM_WAIT and re-emerges
// wherever it was actually headed. Writes are unaffected -- a write
// commits correctly as long as the address is stable by the write cycle,
// which the existing one-cycle-ahead address-setting pattern already
// guarantees; only READS needed this change.
//
// Net effect: every instruction gets +1 cycle for the opcode fetch alone
// (S_FETCH now always detours through S_MEM_WAIT before S_DECODE can
// trust mem_data_in), plus +1 more cycle for every additional memory
// byte an instruction reads. Writes-only instructions (PUSH, LD (HL),r8)
// only pay the universal +1.
//
// Conditional jumps, PUSH AF/POP AF, HALT wake-up remain unsupported.

module cpu_control (
    input  wire        clk,
    input  wire        rst,

    output reg  [15:0] mem_addr,
    input  wire [7:0]  mem_data_in,
    output wire        mem_write_en,
    output wire [7:0]  mem_write_data,

    output wire [15:0] pc,
    output wire [7:0]  ir,
    output reg         unimplemented,
    output wire        halted
);

    localparam ALU_ADD = 3'b000;
    localparam ALU_SUB = 3'b010;
    localparam ALU_CP  = 3'b111;
    localparam SEL_A   = 3'b111;

    // 5 bits now -- S_MEM_WAIT is the 17th state, one past what 4 bits
    // (16 values) could hold.
    localparam S_FETCH          = 5'b00000,
               S_DECODE         = 5'b00001,
               S_MEM_READ       = 5'b00010,
               S_MEM_READ2      = 5'b00011,
               S_EXECUTE        = 5'b00100,
               S_MEM_WRITE      = 5'b00101,
               S_HALT           = 5'b00110,
               S_REG_WRITE_LOW  = 5'b00111,
               S_PUSH_HI        = 5'b01000,
               S_PUSH_LO        = 5'b01001,
               S_POP_LO         = 5'b01010,
               S_POP_HI         = 5'b01011,
               S_CALL_PUSH_HI   = 5'b01100,
               S_CALL_PUSH_LO   = 5'b01101,
               S_RET_LO         = 5'b01110,
               S_RET_HI         = 5'b01111,
               S_MEM_WAIT       = 5'b10000;

    reg [4:0]  state;
    reg [4:0]  next_state; // where S_MEM_WAIT should go once the extra settling cycle passes
    reg [15:0] pc_reg;
    reg [7:0]  ir_reg;
    reg [7:0]  imm8_reg;
    reg [7:0]  jp_low_reg;
    reg [15:0] call_target_reg;

    assign pc     = pc_reg;
    assign ir     = ir_reg;
    assign halted = (state == S_HALT);

    // ---- Decode ----
    wire [2:0] alu_op       = ir_reg[5:3];
    wire [2:0] dest_code    = ir_reg[5:3];
    wire [2:0] ld_imm8_dest = ir_reg[5:3];
    wire [2:0] incdec_r8_sel = ir_reg[5:3];
    wire [2:0] reg_code     = ir_reg[2:0];
    wire [1:0] r16_sel      = ir_reg[5:4];

    wire is_block0 = (ir_reg[7:6] == 2'b00);
    wire is_block1 = (ir_reg[7:6] == 2'b01);
    wire is_block2 = (ir_reg[7:6] == 2'b10);
    wire is_halt   = (ir_reg == 8'h76);
    wire is_nop    = (ir_reg == 8'h00);

    wire is_jr       = (ir_reg == 8'h18);
    wire is_jp_imm16 = (ir_reg == 8'hC3);
    wire is_jp_hl    = (ir_reg == 8'hE9);
    wire is_call     = (ir_reg == 8'hCD);
    wire is_ret      = (ir_reg == 8'hC9);

    wire is_push = (ir_reg[7:6] == 2'b11) && (ir_reg[3:0] == 4'b0101);
    wire is_pop  = (ir_reg[7:6] == 2'b11) && (ir_reg[3:0] == 4'b0001);
    wire is_push_supported = is_push && (r16_sel != 2'b11);
    wire is_pop_supported  = is_pop  && (r16_sel != 2'b11);

    wire is_inc_r8 = is_block0 && (ir_reg[2:0] == 3'b100);
    wire is_dec_r8 = is_block0 && (ir_reg[2:0] == 3'b101);
    wire is_inc_r8_regwrite = is_inc_r8 && (incdec_r8_sel != 3'b110);
    wire is_dec_r8_regwrite = is_dec_r8 && (incdec_r8_sel != 3'b110);

    wire is_inc_r16 = is_block0 && (ir_reg[3:0] == 4'b0011);
    wire is_dec_r16 = is_block0 && (ir_reg[3:0] == 4'b1011);
    wire is_inc_dec_r16_sp   = (is_inc_r16 || is_dec_r16) && (r16_sel == 2'b11);
    wire is_inc_dec_r16_pair = (is_inc_r16 || is_dec_r16) && (r16_sel != 2'b11);

    wire is_mem_operand = (reg_code  == 3'b110);
    wire is_dest_mem     = (dest_code == 3'b110);

    wire is_ld_imm8          = is_block0 && (reg_code == 3'b110);
    wire is_ld_imm8_dest_mem = is_ld_imm8 && (ld_imm8_dest == 3'b110);

    wire is_ld_r16_imm16      = is_block0 && (ir_reg[3:0] == 4'b0001);
    wire is_ld_r16_imm16_sp   = is_ld_r16_imm16 && (r16_sel == 2'b11);
    wire is_ld_r16_imm16_pair = is_ld_r16_imm16 && (r16_sel != 2'b11);

    wire [2:0] reg16_hi_sel = {r16_sel, 1'b0};
    wire [2:0] reg16_lo_sel = {r16_sel, 1'b1};

    wire is_block1_regwrite = is_block1 && !is_halt && !is_dest_mem;
    wire is_block1_memwrite = is_block1 && !is_halt && is_dest_mem;
    wire is_supported_block1 = is_block1_regwrite || is_block1_memwrite;

    wire is_ld_imm8_regwrite = is_ld_imm8 && !is_ld_imm8_dest_mem;
    wire is_ld_imm8_memwrite = is_ld_imm8_dest_mem;
    wire is_supported_ld_imm8 = is_ld_imm8_regwrite || is_ld_imm8_memwrite;

    wire need_mem_read = (is_block2 || is_block1) && is_mem_operand && !is_halt;

    // Does the READ leg of MEM_READ->EXECUTE need a settle cycle?
    // (is_block1_memwrite and the "nothing memory-related" case do NOT --
    // one's a write, the other never touches memory at all.)
    wire mem_read_before_execute = need_mem_read || is_ld_imm8 || is_jr;

    wire [15:0] jr_offset_ext = {{8{mem_data_in[7]}}, mem_data_in};

    wire [7:0]  reg_a_data;
    wire [7:0]  reg_b_data;
    wire [3:0]  flags_out;
    wire [15:0] hl_pair;
    wire [15:0] bc_pair;
    wire [15:0] de_pair;
    wire [15:0] sp_current;

    wire [7:0] resolved_b = is_mem_operand ? mem_data_in : reg_b_data;

    wire [2:0] read_sel_a_mux = (is_inc_r8_regwrite || is_dec_r8_regwrite) ?
                                 incdec_r8_sel : SEL_A;
    wire [2:0] alu_op_mux = is_inc_r8_regwrite ? ALU_ADD :
                            is_dec_r8_regwrite ? ALU_SUB : alu_op;
    wire [7:0] alu_b_mux = (is_inc_r8_regwrite || is_dec_r8_regwrite) ?
                            8'h01 : resolved_b;

    wire [7:0] alu_result;
    wire       alu_z, alu_n, alu_h, alu_c;

    wire preserve_carry = is_inc_r8_regwrite || is_dec_r8_regwrite;
    wire flags_carry_bit = preserve_carry ? flags_out[0] : alu_c;

    wire do_writeback   = is_block2 && (alu_op != ALU_CP);
    wire do_flags_write = is_block2 || is_inc_r8_regwrite || is_dec_r8_regwrite;

    wire [15:0] incdec_pair_value = (r16_sel == 2'b00) ? bc_pair :
                                    (r16_sel == 2'b01) ? de_pair : hl_pair;
    wire [15:0] incdec_pair_result = is_inc_r16 ? (incdec_pair_value + 16'd1) :
                                                   (incdec_pair_value - 16'd1);

    wire in_execute       = (state == S_EXECUTE);
    wire in_reg_write_low = (state == S_REG_WRITE_LOW);
    wire in_push_hi = (state == S_PUSH_HI);
    wire in_push_lo = (state == S_PUSH_LO);
    wire in_pop_lo  = (state == S_POP_LO);
    wire in_pop_hi  = (state == S_POP_HI);
    wire in_call_push_hi = (state == S_CALL_PUSH_HI);
    wire in_call_push_lo = (state == S_CALL_PUSH_LO);
    wire in_ret_lo = (state == S_RET_LO);
    wire in_ret_hi = (state == S_RET_HI);

    wire [15:0] push_pair_value = (r16_sel == 2'b00) ? bc_pair :
                                  (r16_sel == 2'b01) ? de_pair : hl_pair;
    wire [7:0] push_hi_byte = push_pair_value[15:8];
    wire [7:0] push_lo_byte = push_pair_value[7:0];

    wire [2:0] write_sel_mux =
        (in_pop_lo || in_reg_write_low) ? reg16_lo_sel :
        (in_pop_hi || is_ld_r16_imm16_pair || is_inc_dec_r16_pair) ? reg16_hi_sel :
        (is_inc_r8_regwrite || is_dec_r8_regwrite) ? incdec_r8_sel :
        is_ld_imm8_regwrite              ? ld_imm8_dest :
        is_block1_regwrite               ? dest_code    : SEL_A;

    wire [7:0] write_data_mux =
        (in_pop_lo || in_pop_hi)   ? mem_data_in :
        in_reg_write_low            ? jp_low_reg :
        is_inc_dec_r16_pair          ? incdec_pair_result[15:8] :
        is_ld_r16_imm16_pair          ? mem_data_in :
        (is_inc_r8_regwrite || is_dec_r8_regwrite) ? alu_result :
        is_ld_imm8_regwrite            ? mem_data_in :
        is_block1_regwrite               ? resolved_b : alu_result;

    wire rf_write_en =
        (in_execute && (do_writeback || is_block1_regwrite ||
                        is_ld_imm8_regwrite || is_ld_r16_imm16_pair ||
                        is_inc_r8_regwrite || is_dec_r8_regwrite ||
                        is_inc_dec_r16_pair))
        || in_reg_write_low || in_pop_lo || in_pop_hi;

    wire flags_write_en_w = in_execute && do_flags_write;

    wire sp_write_en_w =
        (in_execute && (is_ld_r16_imm16_sp || is_inc_dec_r16_sp)) ||
        in_push_hi || in_push_lo || in_pop_lo || in_pop_hi ||
        in_call_push_hi || in_call_push_lo || in_ret_lo || in_ret_hi;

    wire [15:0] sp_write_data_w =
        (in_execute && is_ld_r16_imm16_sp) ? {mem_data_in, jp_low_reg} :
        (in_execute && is_inc_dec_r16_sp)   ? (is_inc_r16 ? sp_current + 16'd1 : sp_current - 16'd1) :
        (in_push_hi || in_push_lo || in_call_push_hi || in_call_push_lo) ? (sp_current - 16'd1) :
        (in_pop_lo || in_pop_hi || in_ret_lo || in_ret_hi) ? (sp_current + 16'd1) : 16'h0000;

    wire mem_write_en_block1 = in_execute && is_block1_memwrite;
    wire mem_write_en_imm8   = (state == S_MEM_WRITE);
    assign mem_write_en = mem_write_en_block1 || mem_write_en_imm8 ||
                           in_push_hi || in_push_lo ||
                           in_call_push_hi || in_call_push_lo;
    assign mem_write_data = mem_write_en_imm8 ? imm8_reg :
                             in_push_hi          ? push_hi_byte :
                             in_push_lo           ? push_lo_byte :
                             in_call_push_hi       ? pc_reg[15:8] :
                             in_call_push_lo        ? pc_reg[7:0]  : reg_b_data;

    register_file rf (
        .clk(clk), .rst(rst),
        .write_en(rf_write_en), .write_sel(write_sel_mux), .write_data(write_data_mux),
        .read_sel_a(read_sel_a_mux), .read_sel_b(reg_code),
        .read_data_a(reg_a_data), .read_data_b(reg_b_data),
        .flags_write_en(flags_write_en_w),
        .flags_in({alu_z, alu_n, alu_h, flags_carry_bit}),
        .flags_out(flags_out),
        .sp_write_en(sp_write_en_w), .sp_write_data(sp_write_data_w), .sp(sp_current),
        .bc(bc_pair), .de(de_pair), .hl(hl_pair), .af()
    );

    alu al (
        .op(alu_op_mux), .a(reg_a_data), .b(alu_b_mux), .carry_in(flags_out[0]),
        .result(alu_result), .flag_z(alu_z), .flag_n(alu_n),
        .flag_h(alu_h), .flag_c(alu_c)
    );

    always @(posedge clk) begin
        if (rst) begin
            state           <= S_FETCH;
            next_state      <= S_FETCH;
            pc_reg          <= 16'h0000;
            ir_reg          <= 8'h00;
            mem_addr        <= 16'h0000;
            imm8_reg        <= 8'h00;
            jp_low_reg      <= 8'h00;
            call_target_reg <= 16'h0000;
            unimplemented   <= 1'b0;
        end else begin
            unimplemented <= 1'b0;

            case (state)
                S_FETCH: begin
                    mem_addr   <= pc_reg;
                    next_state <= S_DECODE; // opcode fetch always needs the settle cycle
                    state      <= S_MEM_WAIT;
                end

                S_MEM_WAIT: begin
                    // Pure delay -- mem_addr stays whatever it already
                    // was, giving memory a full cycle to settle before
                    // next_state trusts mem_data_in.
                    state <= next_state;
                end

                S_DECODE: begin
                    ir_reg <= mem_data_in; // now genuinely settled
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
                    end else if (is_jp_imm16 || is_ld_r16_imm16 || is_call) begin
                        mem_addr <= pc_reg;
                    end else if (is_push_supported) begin
                        mem_addr <= sp_current - 16'd1;
                    end else if (is_pop_supported) begin
                        mem_addr <= sp_current;
                    end else if (is_ret) begin
                        mem_addr <= sp_current;
                    end

                    if (is_push_supported) begin
                        state <= S_PUSH_HI; // write-only, no settle needed
                    end else if (is_pop_supported) begin
                        next_state <= S_POP_LO; // reads -- needs settle
                        state      <= S_MEM_WAIT;
                    end else if (is_ret) begin
                        next_state <= S_RET_LO; // reads -- needs settle
                        state      <= S_MEM_WAIT;
                    end else if (is_jp_imm16 || is_ld_r16_imm16 || is_call) begin
                        next_state <= S_MEM_READ2; // reads low byte -- needs settle
                        state      <= S_MEM_WAIT;
                    end else if (is_block1_memwrite) begin
                        state <= S_EXECUTE; // write-only, no settle needed
                    end else if (mem_read_before_execute) begin
                        next_state <= S_EXECUTE; // reads operand -- needs settle
                        state      <= S_MEM_WAIT;
                    end else begin
                        state <= S_EXECUTE; // no memory involved at all
                    end
                end

                S_MEM_READ2: begin
                    jp_low_reg <= mem_data_in; // settled low byte
                    mem_addr   <= pc_reg + 1'b1;
                    pc_reg     <= pc_reg + 1'b1;
                    next_state <= S_EXECUTE; // reads high byte -- needs settle
                    state      <= S_MEM_WAIT;
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
                        pc_reg <= pc_reg + 1'b1;
                    end else if (is_call) begin
                        pc_reg          <= pc_reg + 1'b1;
                        call_target_reg <= {mem_data_in, jp_low_reg};
                        mem_addr        <= sp_current - 16'd1;
                    end

                    if (is_ld_imm8_memwrite) begin
                        imm8_reg <= mem_data_in;
                        mem_addr <= hl_pair;
                    end

                    if (is_inc_dec_r16_pair) begin
                        jp_low_reg <= incdec_pair_result[7:0];
                    end

                    unimplemented <= !is_block2 && !is_supported_block1 &&
                                      !is_halt && !is_supported_ld_imm8 &&
                                      !is_jr && !is_jp_imm16 && !is_jp_hl &&
                                      !is_ld_r16_imm16 && !is_nop &&
                                      !is_push_supported && !is_pop_supported &&
                                      !is_call &&
                                      !is_inc_r8_regwrite && !is_dec_r8_regwrite &&
                                      !is_inc_dec_r16_pair && !is_inc_dec_r16_sp;

                    if (is_ld_imm8_memwrite) begin
                        state <= S_MEM_WRITE; // write-only, no settle needed
                    end else if (is_ld_r16_imm16_pair || is_inc_dec_r16_pair) begin
                        state <= S_REG_WRITE_LOW; // writes an already-known byte
                    end else if (is_call) begin
                        state <= S_CALL_PUSH_HI; // write-only, no settle needed
                    end else begin
                        state <= is_halt ? S_HALT : S_FETCH;
                    end
                end

                S_MEM_WRITE: begin
                    state <= S_FETCH;
                end

                S_REG_WRITE_LOW: begin
                    state <= S_FETCH;
                end

                S_PUSH_HI: begin
                    mem_addr <= sp_current - 16'd2;
                    state    <= S_PUSH_LO;
                end

                S_PUSH_LO: begin
                    state <= S_FETCH;
                end

                S_POP_LO: begin
                    // Low byte writes to the register this cycle (mem_data_in
                    // is now settled, thanks to the wait before this state).
                    mem_addr   <= sp_current + 16'd1;
                    next_state <= S_POP_HI; // reads high byte -- needs settle
                    state      <= S_MEM_WAIT;
                end

                S_POP_HI: begin
                    state <= S_FETCH;
                end

                S_CALL_PUSH_HI: begin
                    mem_addr <= sp_current - 16'd2;
                    state    <= S_CALL_PUSH_LO;
                end

                S_CALL_PUSH_LO: begin
                    pc_reg <= call_target_reg;
                    state  <= S_FETCH;
                end

                S_RET_LO: begin
                    jp_low_reg <= mem_data_in; // settled low byte
                    mem_addr   <= sp_current + 16'd1;
                    next_state <= S_RET_HI; // reads high byte -- needs settle
                    state      <= S_MEM_WAIT;
                end

                S_RET_HI: begin
                    pc_reg <= {mem_data_in, jp_low_reg}; // settled high byte
                    state  <= S_FETCH;
                end

                S_HALT: begin
                    state <= S_HALT;
                end

                default: state <= S_FETCH;
            endcase
        end
    end

endmodule
