// tb_cpu_control.v
// Icarus Verilog testbench for cpu_control (Blocks 2, 1, 0-LD-imm8, now with
// memory writes for LD (HL),r8 and LD (HL),d8).

`timescale 1ns/1ps

module tb_cpu_control;

    reg clk = 0;
    reg rst = 1;

    wire [15:0] mem_addr;
    reg  [7:0]  mem_data_in;
    wire        mem_write_en;
    wire [7:0]  mem_write_data;
    wire [15:0] pc;
    wire [7:0]  ir;
    wire        unimplemented;
    wire        halted;

    reg [7:0] mem [0:65535];

    integer errors = 0;

    cpu_control dut (
        .clk(clk), .rst(rst),
        .mem_addr(mem_addr), .mem_data_in(mem_data_in),
        .mem_write_en(mem_write_en), .mem_write_data(mem_write_data),
        .pc(pc), .ir(ir), .unimplemented(unimplemented), .halted(halted)
    );

    // Combinational read
    always @(*) begin
        mem_data_in = mem[mem_addr];
    end

    // Synchronous write -- same convention as register_file's own write port
    always @(posedge clk) begin
        if (mem_write_en) begin
            mem[mem_addr] <= mem_write_data;
        end
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

    // LD (HL),d8 needs an extra cycle (S_MEM_WRITE) beyond the usual 4,
    // so it gets its own wait count rather than reusing run_and_check.
    task run_and_check_5cyc(
        input [7:0] exp_a, input exp_z, input exp_n, input exp_h, input exp_c,
        input exp_unimpl, input [16*8-1:0] name
    );
        begin
            repeat (5) @(posedge clk);
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

        mem[0]  = 8'h81; // ADD A,C
        mem[1]  = 8'h92; // SUB A,D
        mem[2]  = 8'h86; // ADD A,(HL)
        mem[3]  = 8'hBF; // CP A,A
        mem[4]  = 8'h41; // LD B,C
        mem[5]  = 8'h7E; // LD A,(HL)
        mem[6]  = 8'h70; // LD (HL),B  -- NOW SUPPORTED (writes memory)
        mem[7]  = 8'h06; // LD B,d8
        mem[8]  = 8'h42; //   immediate: 0x42
        mem[9]  = 8'h3E; // LD A,d8
        mem[10] = 8'h99; //   immediate: 0x99
        mem[11] = 8'h36; // LD (HL),d8 -- NOW SUPPORTED (writes memory)
        mem[12] = 8'h55; //   immediate: 0x55
        mem[13] = 8'h76; // HALT
        mem[14] = 8'h00; // should never actually be fetched

        mem[16'h0100] = 8'h07; // data operand for the (HL) instructions

        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;

        dut.rf.a_reg = 8'h05;
        dut.rf.c_reg = 8'h03;
        dut.rf.d_reg = 8'h02;
        dut.rf.h_reg = 8'h01;
        dut.rf.l_reg = 8'h00;

        run_and_check(8'h08, 0, 0, 0, 0, 1'b0, "ADD A,C");
        run_and_check(8'h06, 0, 1, 0, 0, 1'b0, "SUB A,D");
        run_and_check(8'h0D, 0, 0, 0, 0, 1'b0, "ADD A,(HL)");
        run_and_check(8'h0D, 1, 1, 0, 0, 1'b0, "CP A,A");

        run_and_check(8'h0D, 1, 1, 0, 0, 1'b0, "LD B,C (A/flags unchanged)");
        if (dut.rf.b_reg !== 8'h03) begin
            $display("FAIL: LD B,C -- B expected 03 got %02h", dut.rf.b_reg);
            errors = errors + 1;
        end

        run_and_check(8'h07, 1, 1, 0, 0, 1'b0, "LD A,(HL)");

        // LD (HL),B: NOW writes memory. B(=0x03) should land at mem[0x0100].
        run_and_check(8'h07, 1, 1, 0, 0, 1'b0, "LD (HL),B");
        if (mem[16'h0100] !== 8'h03) begin
            $display("FAIL: LD (HL),B -- mem[0x0100] expected 03 got %02h", mem[16'h0100]);
            errors = errors + 1;
        end

        run_and_check(8'h07, 1, 1, 0, 0, 1'b0, "LD B,d8");
        if (dut.rf.b_reg !== 8'h42) begin
            $display("FAIL: LD B,d8 -- B expected 42 got %02h", dut.rf.b_reg);
            errors = errors + 1;
        end

        run_and_check(8'h99, 1, 1, 0, 0, 1'b0, "LD A,d8");

        // LD (HL),d8: NOW writes memory, and takes 5 cycles (extra
        // S_MEM_WRITE state), not 4.
        run_and_check_5cyc(8'h99, 1, 1, 0, 0, 1'b0, "LD (HL),d8");
        if (mem[16'h0100] !== 8'h55) begin
            $display("FAIL: LD (HL),d8 -- mem[0x0100] expected 55 got %02h", mem[16'h0100]);
            errors = errors + 1;
        end
        if (dut.rf.b_reg !== 8'h42) begin
            $display("FAIL: LD (HL),d8 -- B should be untouched, expected 42 got %02h", dut.rf.b_reg);
            errors = errors + 1;
        end

        // HALT: A/flags untouched, NOT flagged unimplemented
        run_and_check(8'h99, 1, 1, 0, 0, 1'b0, "HALT");

        if (halted !== 1'b1) begin
            $display("FAIL: halted expected 1 got %b", halted);
            errors = errors + 1;
        end

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
