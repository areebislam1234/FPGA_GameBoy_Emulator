// tb_cpu_control.v
// Icarus Verilog testbench for cpu_control.
//
// Provides a flat behavioral memory (used as both instruction and data
// memory -- fine for this test, real integration may separate them) and
// runs a short hand-written program through the FSM, checking register A,
// the flags, and the `unimplemented` flag after every instruction.
//
// Initial register values are set via hierarchical procedural assignment
// straight into register_file's internal regs -- this FSM can't yet
// execute LD instructions to set up test state itself (that's Block 0/1,
// not built yet), so this is the simulation-only way to seed registers.

`timescale 1ns/1ps

module tb_cpu_control;

    reg clk = 0;
    reg rst = 1;

    wire [15:0] mem_addr;
    reg  [7:0]  mem_data_in;
    wire [15:0] pc;
    wire [7:0]  ir;
    wire        unimplemented;

    reg [7:0] mem [0:65535];

    integer errors = 0;

    cpu_control dut (
        .clk(clk), .rst(rst),
        .mem_addr(mem_addr), .mem_data_in(mem_data_in),
        .pc(pc), .ir(ir), .unimplemented(unimplemented)
    );

    // Combinational memory read model
    always @(*) begin
        mem_data_in = mem[mem_addr];
    end

    always #5 clk = ~clk;

    // Waits exactly one instruction's worth of cycles (FETCH/DECODE/
    // MEM_READ/EXECUTE), then checks A, flags, and unimplemented.
    task run_and_check(
        input [7:0] exp_a, input exp_z, input exp_n, input exp_h, input exp_c,
        input exp_unimpl, input [16*8-1:0] name
    );
        begin
            repeat (4) @(posedge clk);
            #1;
            if (dut.rf.a_reg !== exp_a) begin
                $display("FAIL: %0s -- A expected %02h got %02h", name, exp_a, dut.rf.a_reg);
                errors = errors + 1;
            end
            if (dut.rf.f_reg !== {exp_z, exp_n, exp_h, exp_c}) begin
                $display("FAIL: %0s -- flags expected z=%b n=%b h=%b c=%b got %04b",
                          name, exp_z, exp_n, exp_h, exp_c, dut.rf.f_reg);
                errors = errors + 1;
            end
            if (unimplemented !== exp_unimpl) begin
                $display("FAIL: %0s -- unimplemented expected %b got %b",
                          name, exp_unimpl, unimplemented);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("cpu_control.vcd");
        $dumpvars(0, tb_cpu_control);

        // Program: exercises ADD, SUB, ADD-from-(HL), CP (no write-back),
        // then a non-Block-2 opcode to confirm `unimplemented` fires.
        mem[0] = 8'h81; // ADD A,C
        mem[1] = 8'h92; // SUB A,D
        mem[2] = 8'h86; // ADD A,(HL)
        mem[3] = 8'hBF; // CP A,A
        mem[4] = 8'h00; // NOP (Block 0 -- not yet implemented)

        // Data operand for the ADD A,(HL) instruction
        mem[16'h0100] = 8'h07;

        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;

        // Seed initial register state directly (no LD instructions yet).
        // This MUST happen after rst is fully deasserted -- presetting
        // beforehand gets silently wiped by the next reset-driven clock
        // edge, since register_file's own reset logic is still firing.
        dut.rf.a_reg = 8'h05;
        dut.rf.c_reg = 8'h03;
        dut.rf.d_reg = 8'h02;
        dut.rf.h_reg = 8'h01;
        dut.rf.l_reg = 8'h00;

        // ADD A,C: 0x05 + 0x03 = 0x08
        run_and_check(8'h08, 0, 0, 0, 0, 1'b0, "ADD A,C");

        // SUB A,D: 0x08 - 0x02 = 0x06
        run_and_check(8'h06, 0, 1, 0, 0, 1'b0, "SUB A,D");

        // ADD A,(HL): 0x06 + mem[0x0100]=0x07 = 0x0D
        run_and_check(8'h0D, 0, 0, 0, 0, 1'b0, "ADD A,(HL)");

        // CP A,A: equal, so Z=1, but A must NOT change from 0x0D
        run_and_check(8'h0D, 1, 1, 0, 0, 1'b0, "CP A,A");

        // NOP: not Block 2 -- A/flags untouched, unimplemented should pulse
        run_and_check(8'h0D, 1, 1, 0, 0, 1'b1, "NOP (unimplemented)");

        if (errors == 0)
            $display("PASSED: all cpu_control checks correct");
        else
            $display("FAILED: %0d error(s) found", errors);

        $finish;
    end

endmodule
