// cpu_control.v
// First slice of the SM83 control FSM.
//
// Fully decodes and executes Block 2 (opcode[7:6] == 2'b10) -- the 8-bit
// ALU-with-register instructions (ADD/ADC/SUB/SBC/AND/XOR/OR/CP A,r8),
// including the (HL) memory-operand case. Every other opcode is treated as
// an explicit no-op, with `unimplemented` pulsing high for one cycle so a
// testbench (or later integration) can tell "executed a NOP" apart from
// "hit an opcode this FSM doesn't handle yet" -- silently doing nothing
// would be much harder to debug than an explicit flag.
//
// Memory interface is modeled as combinational read: mem_data_in is
// expected to reflect memory[mem_addr] within the same cycle. Real block
// RAM is registered with a 1-cycle read latency -- FETCH and MEM_READ are
// deliberately separate states specifically so this FSM's timing survives
// that change later (Phase 7 integration) without restructuring.
//
// Also worth knowing: every instruction here takes a fixed 4 cycles
// (FETCH/DECODE/MEM_READ/EXECUTE), even register-only ops that don't
// strictly need the MEM_READ cycle. Real SM83 timing varies per
// instruction -- this trades cycle-exact accuracy for a simpler first
// pass. Worth revisiting once Phase 3's timing-sensitive tests are in play.

module cpu_control (
    input  wire        clk,
    input  wire        rst,

    // Memory interface
    output reg  [15:0] mem_addr,
    input  wire [7:0]  mem_data_in,

    // Observability -- useful for testbenches and later debugging
    output wire [15:0] pc,
    output wire [7:0]  ir,
    output reg         unimplemented   // pulses 1 cycle on any non-Block-2 opcode
);

    // Must match alu.v / register_file.v encodings
    localparam ALU_CP = 3'b111;
    localparam SEL_A  = 3'b111;

    localparam S_FETCH    = 2'b00,
               S_DECODE   = 2'b01,
               S_MEM_READ = 2'b10,
               S_EXECUTE  = 2'b11;

    reg [1:0]  state;
    reg [15:0] pc_reg;
    reg [7:0]  ir_reg;

    assign pc = pc_reg;
    assign ir = ir_reg;

    wire [2:0] alu_op        = ir_reg[5:3];
    wire [2:0] reg_code      = ir_reg[2:0];
    wire       is_block2     = (ir_reg[7:6] == 2'b10);
    wire       is_mem_operand = (reg_code == 3'b110);

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
    wire do_flags_write = is_block2;

    // These MUST be combinational, not registered -- register_file's own
    // posedge-triggered write needs write_en high for the entire cycle
    // alu_result is valid (i.e. the whole S_EXECUTE state), not a pulse
    // that arrives one edge late relative to when the ALU actually computed.
    wire in_execute          = (state == S_EXECUTE);
    wire rf_write_en         = in_execute && do_writeback;
    wire flags_write_en_reg  = in_execute && do_flags_write;

    register_file rf (
        .clk(clk), .rst(rst),
        .write_en(rf_write_en), .write_sel(SEL_A), .write_data(alu_result),
        .read_sel_a(SEL_A), .read_sel_b(reg_code),
        .read_data_a(reg_a_data), .read_data_b(reg_b_data),
        .flags_write_en(flags_write_en_reg),
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
            state              <= S_FETCH;
            pc_reg             <= 16'h0000;
            ir_reg             <= 8'h00;
            mem_addr           <= 16'h0000;
            unimplemented      <= 1'b0;
        end else begin
            unimplemented      <= 1'b0; // default; only pulses in EXECUTE below

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
                    // is_block2/is_mem_operand are trustworthy this cycle.
                    if (is_block2 && is_mem_operand) begin
                        mem_addr <= hl_pair;
                    end
                    state <= S_EXECUTE;
                end

                S_EXECUTE: begin
                    unimplemented <= !is_block2;
                    state         <= S_FETCH;
                end

                default: state <= S_FETCH;
            endcase
        end
    end

endmodule
