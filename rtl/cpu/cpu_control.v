// cpu_control.v
// SM83 control FSM. Supports:
//   Block 2: ALU A,r8       Block 1: LD r8,r8' (incl. LD (HL),r8)
//   Block 0: LD r8,imm8 (incl. LD (HL),d8), LD r16,imm16, NOP
//   Unconditional control flow: JR imm8, JP imm16, JP (HL)
//   PUSH/POP BC,DE,HL  <-- NEW this round
//
// PUSH AF / POP AF are deliberately NOT supported yet: F (the flags
// register) has its own dedicated write port in register_file, entirely
// different from how B/C/D/E/H/L get written, so folding it into this
// same change would tangle two different write mechanisms together.
// Flagged as unimplemented, same treatment as conditional jumps.
//
// PUSH/POP are the most stateful instructions built so far -- each needs
// TWO sequential memory operations (write high-then-low for PUSH, read
// low-then-high for POP) at two DIFFERENT addresses, with SP decrementing
// or incrementing by 1 between each step. That's 4 new states:
// S_PUSH_HI, S_PUSH_LO, S_POP_LO, S_POP_HI.
//
// PUSH stores high byte at SP-1, low byte at SP-2, finishing with SP
// decremented by 2 total -- so after a PUSH, SP points at the low byte,
// matching the little-endian convention used everywhere else in this
// design. POP reverses this exactly: low byte first (at current SP),
// then high byte (at SP+1), SP ending incremented by 2.
//
// Conditional jumps are still NOT supported. Memory interface:
// combinational read, synchronous write. HALT parks with no wake-up yet.

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

    localparam ALU_CP = 3'b111;
    localparam SEL_A  = 3'b111;

    // Now 4 bits wide -- 8 states wasn't enough room once PUSH/POP added
    // 4 more on top of the existing 8.
    localparam S_FETCH         = 4'b0000,
               S_DECODE        = 4'b0001,
               S_MEM_READ      = 4'b0010,
               S_MEM_READ2     = 4'b0011,
               S_EXECUTE       = 4'b0100,
               S_MEM_WRITE     = 4'b0101,
               S_HALT          = 4'b0110,
               S_REG_WRITE_LOW = 4'b0111,
               S_PUSH_HI       = 4'b1000,
               S_PUSH_LO       = 4'b1001,
               S_POP_LO        = 4'b1010,
               S_POP_HI        = 4'b1011;

    reg [3:0]  state;
    reg [15:0] pc_reg;
    reg [7:0]  ir_reg;
    reg [7:0]  imm8_reg;
    reg [7:0]  jp_low_reg;

    assign pc     = pc_reg;
    assign ir     = ir_reg;
    assign halted = (state == S_HALT);

    // ---- Decode ----
    wire [2:0] alu_op       = ir_reg[5:3];
    wire [2:0] dest_code    = ir_reg[5:3];
    wire [2:0] ld_imm8_dest = ir_reg[5:3];
    wire [2:0] reg_code     = ir_reg[2:0];
    wire [1:0] r16_sel      = ir_reg[5:4]; // shared by LD r16,imm16 AND PUSH/POP --
                                             // same bit position, same BC/DE/HL
                                             // encoding in both instruction tables.

    wire is_block0 = (ir_reg[7:6] == 2'b00);
    wire is_block1 = (ir_reg[7:6] == 2'b01);
    wire is_block2 = (ir_reg[7:6] == 2'b10);
    wire is_halt   = (ir_reg == 8'h76);
    wire is_nop    = (ir_reg == 8'h00);

    wire is_jr       = (ir_reg == 8'h18);
    wire is_jp_imm16 = (ir_reg == 8'hC3);
    wire is_jp_hl    = (ir_reg == 8'hE9);

    wire is_push = (ir_reg[7:6] == 2'b11) && (ir_reg[3:0] == 4'b0101); // 11 qq 0101
    wire is_pop  = (ir_reg[7:6] == 2'b11) && (ir_reg[3:0] == 4'b0001); // 11 qq 0001
    wire is_push_supported = is_push && (r16_sel != 2'b11); // excludes PUSH AF
    wire is_pop_supported  = is_pop  && (r16_sel != 2'b11); // excludes POP AF

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

    wire [15:0] jr_offset_ext = {{8{mem_data_in[7]}}, mem_data_in};

    wire [7:0]  reg_a_data;
    wire [7:0]  reg_b_data;
    wire [3:0]  flags_out;
    wire [15:0] hl_pair;
    wire [15:0] bc_pair;
    wire [15:0] de_pair;
    wire [15:0] sp_current;

    wire [7:0] resolved_b = is_mem_operand ? mem_data_in : reg_b_data;

    wire [7:0] alu_result;
    wire       alu_z, alu_n, alu_h, alu_c;

    wire do_writeback   = is_block2 && (alu_op != ALU_CP);
    wire do_flags_write = is_block2;

    wire in_execute       = (state == S_EXECUTE);
    wire in_reg_write_low = (state == S_REG_WRITE_LOW);
    wire in_push_hi = (state == S_PUSH_HI);
    wire in_push_lo = (state == S_PUSH_LO);
    wire in_pop_lo  = (state == S_POP_LO);
    wire in_pop_hi  = (state == S_POP_HI);

    // Which pair PUSH is reading -- r16_sel==2'b10 (HL) is the only
    // remaining option once is_push_supported has already excluded 2'b11.
    wire [15:0] push_pair_value = (r16_sel == 2'b00) ? bc_pair :
                                  (r16_sel == 2'b01) ? de_pair : hl_pair;
    wire [7:0] push_hi_byte = push_pair_value[15:8];
    wire [7:0] push_lo_byte = push_pair_value[7:0];

    // Register-file 8-bit write port mux. POP writes low byte first (its
    // own state), then high byte (its own state) -- opposite order from
    // LD r16,imm16, which is why these are separate branches rather than
    // shared with in_reg_write_low even though the underlying reg16_*_sel
    // wires are reused.
    wire [2:0] write_sel_mux =
        (in_pop_lo || in_reg_write_low) ? reg16_lo_sel :
        (in_pop_hi || is_ld_r16_imm16_pair) ? reg16_hi_sel :
        is_ld_imm8_regwrite              ? ld_imm8_dest :
        is_block1_regwrite               ? dest_code    : SEL_A;

    wire [7:0] write_data_mux =
        (in_pop_lo || in_pop_hi)   ? mem_data_in :
        in_reg_write_low            ? jp_low_reg :
        is_ld_r16_imm16_pair         ? mem_data_in :
        is_ld_imm8_regwrite           ? mem_data_in :
        is_block1_regwrite             ? resolved_b : alu_result;

    wire rf_write_en =
        (in_execute && (do_writeback || is_block1_regwrite ||
                        is_ld_imm8_regwrite || is_ld_r16_imm16_pair))
        || in_reg_write_low || in_pop_lo || in_pop_hi;

    wire flags_write_en_w = in_execute && do_flags_write;

    // SP write port -- three different reasons to write it, unified here.
    // LD SP,imm16 sets it directly; PUSH/POP each adjust it by 1 per
    // sub-state (decrementing twice for PUSH, incrementing twice for POP),
    // using sp_current's value AS OF that specific cycle -- since the
    // first adjustment's write has already committed by the time the
    // second sub-state runs, "current minus/plus 1" naturally accumulates
    // to the correct net -2/+2 without needing to track an original value.
    wire sp_write_en_w =
        (in_execute && is_ld_r16_imm16_sp) || in_push_hi || in_push_lo ||
        in_pop_lo || in_pop_hi;

    wire [15:0] sp_write_data_w =
        (in_execute && is_ld_r16_imm16_sp) ? {mem_data_in, jp_low_reg} :
        (in_push_hi || in_push_lo)          ? (sp_current - 16'd1) :
        (in_pop_lo || in_pop_hi)             ? (sp_current + 16'd1) : 16'h0000;

    wire mem_write_en_block1 = in_execute && is_block1_memwrite;
    wire mem_write_en_imm8   = (state == S_MEM_WRITE);
    assign mem_write_en = mem_write_en_block1 || mem_write_en_imm8 ||
                           in_push_hi || in_push_lo;
    assign mem_write_data = mem_write_en_imm8 ? imm8_reg :
                             in_push_hi          ? push_hi_byte :
                             in_push_lo           ? push_lo_byte : reg_b_data;

    register_file rf (
        .clk(clk), .rst(rst),
        .write_en(rf_write_en), .write_sel(write_sel_mux), .write_data(write_data_mux),
        .read_sel_a(SEL_A), .read_sel_b(reg_code),
        .read_data_a(reg_a_data), .read_data_b(reg_b_data),
        .flags_write_en(flags_write_en_w),
        .flags_in({alu_z, alu_n, alu_h, alu_c}),
        .flags_out(flags_out),
        .sp_write_en(sp_write_en_w), .sp_write_data(sp_write_data_w), .sp(sp_current),
        .bc(bc_pair), .de(de_pair), .hl(hl_pair), .af()
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
                        mem_addr <= pc_reg;
                    end else if (is_push_supported) begin
                        mem_addr <= sp_current - 16'd1; // high byte's address
                    end else if (is_pop_supported) begin
                        mem_addr <= sp_current;          // low byte's address
                    end

                    if (is_push_supported) begin
                        state <= S_PUSH_HI;
                    end else if (is_pop_supported) begin
                        state <= S_POP_LO;
                    end else begin
                        state <= (is_jp_imm16 || is_ld_r16_imm16) ? S_MEM_READ2 : S_EXECUTE;
                    end
                end

                S_MEM_READ2: begin
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
                        pc_reg <= pc_reg + 1'b1;
                    end

                    if (is_ld_imm8_memwrite) begin
                        imm8_reg <= mem_data_in;
                        mem_addr <= hl_pair;
                    end

                    unimplemented <= !is_block2 && !is_supported_block1 &&
                                      !is_halt && !is_supported_ld_imm8 &&
                                      !is_jr && !is_jp_imm16 && !is_jp_hl &&
                                      !is_ld_r16_imm16 && !is_nop &&
                                      !is_push_supported && !is_pop_supported;

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
                    state <= S_FETCH;
                end

                S_PUSH_HI: begin
                    // High byte writes THIS cycle (mem_addr already =
                    // sp_current(original)-1, set during MEM_READ). Prepare
                    // the low byte's address for next cycle, using
                    // sp_current's still-original value here.
                    mem_addr <= sp_current - 16'd2;
                    state    <= S_PUSH_LO;
                end

                S_PUSH_LO: begin
                    state <= S_FETCH;
                end

                S_POP_LO: begin
                    // Low byte writes to the register THIS cycle. Prepare
                    // the high byte's read address for next cycle.
                    mem_addr <= sp_current + 16'd1;
                    state    <= S_POP_HI;
                end

                S_POP_HI: begin
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
