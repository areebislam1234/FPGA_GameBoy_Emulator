// alu.v
// SM83 (Game Boy CPU) 8-bit ALU.
//
// op encoding matches the SM83 instruction set's ALU op field:
//   000=ADD  001=ADC  010=SUB  011=SBC  100=AND  101=XOR  110=OR  111=CP
//
// CP computes the same result/flags as SUB but is expected NOT to be
// written back to a register -- that decision belongs to whatever control
// logic drives this module, not to the ALU itself.

module alu (
    input  wire [2:0] op,
    input  wire [7:0] a,          // accumulator operand
    input  wire [7:0] b,          // second operand
    input  wire       carry_in,   // current C flag, needed by ADC/SBC
    output reg  [7:0] result,
    output reg         flag_z,    // set if result == 0
    output reg         flag_n,    // set for subtract-family ops
    output reg         flag_h,    // half-carry: carry/borrow out of bit 3
    output reg         flag_c     // carry/borrow out of bit 7
);

    localparam ALU_ADD = 3'b000,
               ALU_ADC  = 3'b001,
               ALU_SUB  = 3'b010,
               ALU_SBC  = 3'b011,
               ALU_AND  = 3'b100,
               ALU_XOR  = 3'b101,
               ALU_OR   = 3'b110,
               ALU_CP   = 3'b111;

    reg [8:0] sum9;      // 9 bits: bit 8 catches the carry out of bit 7
    reg [4:0] halfsum;   // 5 bits: bit 4 catches the carry out of bit 3

    always @(*) begin
        result = 8'h00;
        flag_z = 1'b0;
        flag_n = 1'b0;
        flag_h = 1'b0;
        flag_c = 1'b0;
        sum9    = 9'h000;
        halfsum = 5'h00;

        case (op)
            ALU_ADD: begin
                sum9    = {1'b0, a} + {1'b0, b};
                halfsum = {1'b0, a[3:0]} + {1'b0, b[3:0]};
                result  = sum9[7:0];
                flag_c  = sum9[8];
                flag_h  = halfsum[4];
                flag_n  = 1'b0;
                flag_z  = (result == 8'h00);
            end

            ALU_ADC: begin
                sum9    = {1'b0, a} + {1'b0, b} + {8'b0, carry_in};
                halfsum = {1'b0, a[3:0]} + {1'b0, b[3:0]} + {4'b0, carry_in};
                result  = sum9[7:0];
                flag_c  = sum9[8];
                flag_h  = halfsum[4];
                flag_n  = 1'b0;
                flag_z  = (result == 8'h00);
            end

            ALU_SUB: begin
                sum9    = {1'b0, a} - {1'b0, b};
                halfsum = {1'b0, a[3:0]} - {1'b0, b[3:0]};
                result  = sum9[7:0];
                flag_c  = sum9[8];   // set means a borrow occurred
                flag_h  = halfsum[4];
                flag_n  = 1'b1;
                flag_z  = (result == 8'h00);
            end

            ALU_SBC: begin
                sum9    = {1'b0, a} - {1'b0, b} - {8'b0, carry_in};
                halfsum = {1'b0, a[3:0]} - {1'b0, b[3:0]} - {4'b0, carry_in};
                result  = sum9[7:0];
                flag_c  = sum9[8];
                flag_h  = halfsum[4];
                flag_n  = 1'b1;
                flag_z  = (result == 8'h00);
            end

            ALU_AND: begin
                result = a & b;
                flag_z = (result == 8'h00);
                flag_n = 1'b0;
                flag_h = 1'b1;   // AND always sets H on real hardware
                flag_c = 1'b0;
            end

            ALU_XOR: begin
                result = a ^ b;
                flag_z = (result == 8'h00);
                flag_n = 1'b0;
                flag_h = 1'b0;
                flag_c = 1'b0;
            end

            ALU_OR: begin
                result = a | b;
                flag_z = (result == 8'h00);
                flag_n = 1'b0;
                flag_h = 1'b0;
                flag_c = 1'b0;
            end

            ALU_CP: begin
                sum9    = {1'b0, a} - {1'b0, b};
                halfsum = {1'b0, a[3:0]} - {1'b0, b[3:0]};
                result  = sum9[7:0]; // computed for completeness; caller should discard
                flag_c  = sum9[8];
                flag_h  = halfsum[4];
                flag_n  = 1'b1;
                flag_z  = (result == 8'h00);
            end

            default: result = 8'h00;
        endcase
    end

endmodule
