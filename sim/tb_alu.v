// tb_alu.v
// Icarus Verilog testbench for alu.v.

`timescale 1ns/1ps

module tb_alu;

    localparam ALU_ADD = 3'b000, ALU_ADC = 3'b001, ALU_SUB = 3'b010, ALU_SBC = 3'b011,
               ALU_AND = 3'b100, ALU_XOR = 3'b101, ALU_OR  = 3'b110, ALU_CP  = 3'b111;

    reg  [2:0] op;
    reg  [7:0] a, b;
    reg        carry_in;
    wire [7:0] result;
    wire       flag_z, flag_n, flag_h, flag_c;

    integer errors = 0;

    alu dut (
        .op(op), .a(a), .b(b), .carry_in(carry_in),
        .result(result), .flag_z(flag_z), .flag_n(flag_n),
        .flag_h(flag_h), .flag_c(flag_c)
    );

    task check(
        input [2:0] t_op, input [7:0] t_a, input [7:0] t_b, input t_cin,
        input [7:0] exp_result, input exp_z, input exp_n, input exp_h, input exp_c,
        input [16*8-1:0] name
    );
        begin
            op = t_op; a = t_a; b = t_b; carry_in = t_cin;
            #1;
            if (result !== exp_result || flag_z !== exp_z || flag_n !== exp_n ||
                flag_h !== exp_h || flag_c !== exp_c) begin
                $display("FAIL: %0s -- got result=%02h z=%b n=%b h=%b c=%b, expected result=%02h z=%b n=%b h=%b c=%b",
                          name, result, flag_z, flag_n, flag_h, flag_c,
                          exp_result, exp_z, exp_n, exp_h, exp_c);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("alu.vcd");
        $dumpvars(0, tb_alu);

        // ADD: simple case, no carries
        check(ALU_ADD, 8'h02, 8'h03, 1'b0, 8'h05, 0, 0, 0, 0, "ADD simple");
        // ADD: half-carry out of bit 3 (0x0F + 0x01 crosses the nibble boundary)
        check(ALU_ADD, 8'h0F, 8'h01, 1'b0, 8'h10, 0, 0, 1, 0, "ADD half-carry");
        // ADD: full carry out + half-carry, result wraps to zero
        check(ALU_ADD, 8'hFF, 8'h01, 1'b0, 8'h00, 1, 0, 1, 1, "ADD wrap to zero");

        // ADC: carry_in folded into the sum
        check(ALU_ADC, 8'h0E, 8'h01, 1'b1, 8'h10, 0, 0, 1, 0, "ADC with carry-in");

        // SUB: simple case
        check(ALU_SUB, 8'h05, 8'h03, 1'b0, 8'h02, 0, 1, 0, 0, "SUB simple");
        // SUB: half-borrow (0x10 - 0x01 needs to borrow into the low nibble)
        check(ALU_SUB, 8'h10, 8'h01, 1'b0, 8'h0F, 0, 1, 1, 0, "SUB half-borrow");
        // SUB: full borrow, result wraps
        check(ALU_SUB, 8'h00, 8'h01, 1'b0, 8'hFF, 0, 1, 1, 1, "SUB full borrow");

        // SBC: borrow_in folded into the subtraction
        check(ALU_SBC, 8'h05, 8'h03, 1'b1, 8'h01, 0, 1, 0, 0, "SBC with borrow-in");

        // AND always sets H, clears N and C
        check(ALU_AND, 8'hF0, 8'h0F, 1'b0, 8'h00, 1, 0, 1, 0, "AND to zero");
        check(ALU_AND, 8'hFF, 8'h3C, 1'b0, 8'h3C, 0, 0, 1, 0, "AND partial");

        // XOR clears N, H, C
        check(ALU_XOR, 8'hFF, 8'hFF, 1'b0, 8'h00, 1, 0, 0, 0, "XOR to zero");
        check(ALU_XOR, 8'hAA, 8'h55, 1'b0, 8'hFF, 0, 0, 0, 0, "XOR partial");

        // OR clears N, H, C
        check(ALU_OR, 8'h00, 8'h00, 1'b0, 8'h00, 1, 0, 0, 0, "OR to zero");
        check(ALU_OR, 8'hF0, 8'h0F, 1'b0, 8'hFF, 0, 0, 0, 0, "OR partial");

        // CP behaves like SUB for flags -- equal operands set Z
        check(ALU_CP, 8'h05, 8'h05, 1'b0, 8'h00, 1, 1, 0, 0, "CP equal");
        check(ALU_CP, 8'h03, 8'h05, 1'b0, 8'hFE, 0, 1, 1, 1, "CP less-than");

        if (errors == 0)
            $display("PASSED: all ALU checks correct");
        else
            $display("FAILED: %0d error(s) found", errors);

        $finish;
    end

endmodule
