// register_file.v
// SM83 (Game Boy CPU) register file.
//
// Register encoding matches the SM83 instruction set's 3-bit r/r' field:
//   000=B  001=C  010=D  011=E  100=H  101=L  111=A
//   110 = (HL) memory access -- not a physical register, handled outside
//   this module by whatever reads/writes memory.
//
// Exposes the standard 16-bit pairs (BC, DE, HL, AF) since many SM83
// instructions (16-bit loads, PUSH/POP, INC rr/DEC rr) operate on pairs
// rather than single registers.

module register_file (
    input  wire        clk,
    input  wire        rst,

    // Single write port for the 7 general 8-bit registers
    input  wire        write_en,
    input  wire [2:0]  write_sel,
    input  wire [7:0]  write_data,

    // Two independent read ports (e.g. ADD A,B reads A and B in one cycle)
    input  wire [2:0]  read_sel_a,
    input  wire [2:0]  read_sel_b,
    output reg  [7:0]  read_data_a,
    output reg  [7:0]  read_data_b,

    // Flags (F) -- written directly by the ALU, separate from write_sel
    input  wire        flags_write_en,
    input  wire [3:0]  flags_in,    // {Z, N, H, C}
    output wire [3:0]  flags_out,

    // Stack pointer -- dedicated 16-bit port, not part of the r/r' encoding
    input  wire         sp_write_en,
    input  wire [15:0]  sp_write_data,
    output wire [15:0]  sp,

    // Register pairs, composed from the 8-bit registers above
    output wire [15:0] bc,
    output wire [15:0] de,
    output wire [15:0] hl,
    output wire [15:0] af
);

    localparam SEL_B = 3'b000,
               SEL_C = 3'b001,
               SEL_D = 3'b010,
               SEL_E = 3'b011,
               SEL_H = 3'b100,
               SEL_L = 3'b101,
               SEL_A = 3'b111;
               // 3'b110 = (HL) -- intentionally unhandled here

    reg [7:0]  b_reg, c_reg, d_reg, e_reg, h_reg, l_reg, a_reg;
    reg [3:0]  f_reg;   // only the upper nibble (Z N H C) is meaningful
    reg [15:0] sp_reg;

    // Write port
    always @(posedge clk) begin
        if (rst) begin
            b_reg <= 8'h00; c_reg <= 8'h00;
            d_reg <= 8'h00; e_reg <= 8'h00;
            h_reg <= 8'h00; l_reg <= 8'h00;
            a_reg <= 8'h00;
        end else if (write_en) begin
            case (write_sel)
                SEL_B: b_reg <= write_data;
                SEL_C: c_reg <= write_data;
                SEL_D: d_reg <= write_data;
                SEL_E: e_reg <= write_data;
                SEL_H: h_reg <= write_data;
                SEL_L: l_reg <= write_data;
                SEL_A: a_reg <= write_data;
                default: ; // (HL) select -- not a register write here
            endcase
        end
    end

    // Flags -- independent write path from the ALU
    always @(posedge clk) begin
        if (rst) begin
            f_reg <= 4'h0;
        end else if (flags_write_en) begin
            f_reg <= flags_in;
        end
    end

    // Stack pointer
    always @(posedge clk) begin
        if (rst) begin
            sp_reg <= 16'h0000;
        end else if (sp_write_en) begin
            sp_reg <= sp_write_data;
        end
    end

    // Read port A (combinational)
    always @(*) begin
        case (read_sel_a)
            SEL_B: read_data_a = b_reg;
            SEL_C: read_data_a = c_reg;
            SEL_D: read_data_a = d_reg;
            SEL_E: read_data_a = e_reg;
            SEL_H: read_data_a = h_reg;
            SEL_L: read_data_a = l_reg;
            SEL_A: read_data_a = a_reg;
            default: read_data_a = 8'h00; // (HL) -- handled outside
        endcase
    end

    // Read port B (combinational)
    always @(*) begin
        case (read_sel_b)
            SEL_B: read_data_b = b_reg;
            SEL_C: read_data_b = c_reg;
            SEL_D: read_data_b = d_reg;
            SEL_E: read_data_b = e_reg;
            SEL_H: read_data_b = h_reg;
            SEL_L: read_data_b = l_reg;
            SEL_A: read_data_b = a_reg;
            default: read_data_b = 8'h00; // (HL) -- handled outside
        endcase
    end

    assign flags_out = f_reg;
    assign sp         = sp_reg;

    assign bc = {b_reg, c_reg};
    assign de = {d_reg, e_reg};
    assign hl = {h_reg, l_reg};
    assign af = {a_reg, f_reg, 4'h0}; // F's lower nibble is always 0 on real hardware

endmodule
