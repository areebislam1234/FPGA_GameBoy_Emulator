// tb_cpu_control.v
// Icarus Verilog testbench for cpu_control (Block 2 + Block 1).
//
// Provides a flat behavioral memory and runs a short hand-written program
// through the FSM, checking register A, flags, and `unimplemented` after
// every instruction -- plus dedicated checks for the LD-into-a-different-
// register case, the unsupported LD (HL),r case, and HALT.
//
// Initial register values are seeded via hierarchical procedural
// assignment straight into register_file's internal regs, since this FSM
// can't yet execute LD r,imm8 to set up its own test state.

`timescale 1ns/1ps

module tb_cpu_control;

    reg clk = 0;
    reg rst = 1;

    wire [15:0] mem_addr;
    reg  [7:0]  mem_data_in;
    wire [15:0] pc;
    wire [7:0]  ir;
    wire        unimplemented;
    wire        halted;

    reg [7:0] mem [0:65535];

    integer errors = 0;

    cpu_control dut (
        .clk(clk), .rst(rst),
        .mem_addr(mem_addr), .mem_data_in(mem_data_in),
        .pc(pc), .ir(ir), .unimplemented(unimplemented), .halted(halted)
    );

    always @(*) begin
        mem_data_in = mem[mem_addr];
    end

    always #5 clk = ~clk;

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

        mem[0] = 8'h81; // ADD A,C
        mem[1] = 8'h92; // SUB A,D
        mem[2] = 8'h86; // ADD A,(HL)
        mem[3] = 8'hBF; // CP A,A
        mem[4] = 8'h41; // LD B,C
        mem[5] = 8'h7E; // LD A,(HL)
        mem[6] = 8'h70; // LD (HL),B  -- unsupported, expect unimplemented=1
        mem[7] = 8'h76; // HALT
        mem[8] = 8'h00; // should never actually be fetched

        mem[16'h0100] = 8'h07; // data operand for the (HL) instructions

        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;

        // Seed initial register state directly (no LD imm8 support yet)
        dut.rf.a_reg = 8'h05;
        dut.rf.c_reg = 8'h03;
        dut.rf.d_reg = 8'h02;
        dut.rf.h_reg = 8'h01;
        dut.rf.l_reg = 8'h00;

        run_and_check(8'h08, 0, 0, 0, 0, 1'b0, "ADD A,C");
        run_and_check(8'h06, 0, 1, 0, 0, 1'b0, "SUB A,D");
        run_and_check(8'h0D, 0, 0, 0, 0, 1'b0, "ADD A,(HL)");
        run_and_check(8'h0D, 1, 1, 0, 0, 1'b0, "CP A,A");

        // LD B,C: A/flags must be untouched by a plain register move
        run_and_check(8'h0D, 1, 1, 0, 0, 1'b0, "LD B,C (A/flags unchanged)");
        if (dut.rf.b_reg !== 8'h03) begin
            $display("FAIL: LD B,C -- B expected 03 got %02h", dut.rf.b_reg);
            errors = errors + 1;
        end

        // LD A,(HL): A takes the memory byte; flags still untouched (LD never sets them)
        run_and_check(8'h07, 1, 1, 0, 0, 1'b0, "LD A,(HL)");

        // LD (HL),B: unsupported -- nothing should change, unimplemented should fire
        run_and_check(8'h07, 1, 1, 0, 0, 1'b1, "LD (HL),B (unsupported)");

        // HALT: A/flags untouched, NOT flagged unimplemented (it's handled, just parked)
        run_and_check(8'h07, 1, 1, 0, 0, 1'b0, "HALT");

        if (halted !== 1'b1) begin
            $display("FAIL: halted expected 1 got %b", halted);
            errors = errors + 1;
        end

        // Confirm the FSM actually stays parked -- pc must not advance further,
        // and if it incorrectly fetched mem[8] (0x00), unimplemented would pulse.
        begin : halt_hold_check
            reg [15:0] pc_at_halt;
            pc_at_halt = pc;
            repeat (8) @(posedge clk);
            #1;
            if (pc !== pc_at_halt) begin
                $display("FAIL: pc advanced after HALT -- was %04h, now %04h", pc_at_halt, pc);
                errors = errors + 1;
            end
            if (halted !== 1'b1) begin
                $display("FAIL: halted dropped after holding -- got %b", halted);
                errors = errors + 1;
            end
        end

        if (errors == 0)
            $display("PASSED: all cpu_control checks correct");
        else
            $display("FAILED: %0d error(s) found", errors);

        $finish;
    end

endmodule
