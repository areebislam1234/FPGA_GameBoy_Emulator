// tb_cpu_control_jumps.v
// Icarus Verilog testbench for cpu_control's jump support (JR, JP imm16,
// JP (HL)) -- updated for the REGISTERED-read memory timing model.
//
// This file already used generous fixed cycle budgets per phase rather
// than precise per-instruction counts, so the update here is simpler
// than the primary regression file: switch the memory model to
// registered (matching real vram.v), and enlarge the budget to
// comfortably cover the new, longer per-instruction cycle counts.
// Worst-case phase (JP imm16, two reads, + landing LD + HALT) now needs
// roughly 19 cycles -- a budget of 50 leaves generous headroom.

`timescale 1ns/1ps

module tb_cpu_control_jumps;

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

    // REGISTERED read -- matches real vram.v.
    always @(posedge clk) begin
        mem_data_in <= mem[mem_addr];
    end

    always @(posedge clk) begin
        if (mem_write_en) begin
            mem[mem_addr] <= mem_write_data;
        end
    end

    always #5 clk = ~clk;

    task run_phase(input [16*8-1:0] name, input [7:0] exp_a, input exp_unimpl);
        begin
            rst = 1;
            repeat (3) @(negedge clk);
            rst = 0;

            repeat (50) @(posedge clk);
            #1;

            if (dut.rf.a_reg !== exp_a) begin
                $display("FAIL: %0s -- A expected %02h got %02h", name, exp_a, dut.rf.a_reg);
                errors = errors + 1;
            end
            if (halted !== 1'b1) begin
                $display("FAIL: %0s -- expected halted=1, got %b (jump may not have landed correctly)",
                          name, halted);
                errors = errors + 1;
            end
            if (unimplemented !== exp_unimpl) begin
                $display("FAIL: %0s -- unimplemented expected %b got %b", name, exp_unimpl, unimplemented);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("cpu_control_jumps.vcd");
        $dumpvars(0, tb_cpu_control_jumps);

        mem[0] = 8'h18; mem[1] = 8'h03;
        mem[2] = 8'h3E; mem[3] = 8'hFF;
        mem[4] = 8'h00;
        mem[5] = 8'h3E; mem[6] = 8'h11;
        mem[7] = 8'h76;
        run_phase("JR forward", 8'h11, 1'b0);

        mem[0]  = 8'h18; mem[1] = 8'h08;
        mem[2]  = 8'h00;
        mem[3]  = 8'h3E; mem[4] = 8'h22;
        mem[5]  = 8'h76;
        mem[6]  = 8'h00; mem[7] = 8'h00; mem[8] = 8'h00; mem[9] = 8'h00;
        mem[10] = 8'h18; mem[11] = 8'hF7;
        run_phase("JR backward", 8'h22, 1'b0);

        mem[0] = 8'hC3; mem[1] = 8'h20; mem[2] = 8'h00;
        mem[16'h0020] = 8'h3E; mem[16'h0021] = 8'h33;
        mem[16'h0022] = 8'h76;
        run_phase("JP imm16", 8'h33, 1'b0);

        mem[0] = 8'hE9;
        mem[16'h0040] = 8'h3E; mem[16'h0041] = 8'h44;
        mem[16'h0042] = 8'h76;

        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        dut.rf.h_reg = 8'h00;
        dut.rf.l_reg = 8'h40;

        repeat (50) @(posedge clk);
        #1;
        if (dut.rf.a_reg !== 8'h44) begin
            $display("FAIL: JP (HL) -- A expected 44 got %02h", dut.rf.a_reg);
            errors = errors + 1;
        end
        if (halted !== 1'b1) begin
            $display("FAIL: JP (HL) -- expected halted=1, got %b", halted);
            errors = errors + 1;
        end
        if (unimplemented !== 1'b0) begin
            $display("FAIL: JP (HL) -- unimplemented expected 0 got %b", unimplemented);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASSED: all cpu_control jump checks correct");
        else
            $display("FAILED: %0d error(s) found", errors);

        $finish;
    end

endmodule
