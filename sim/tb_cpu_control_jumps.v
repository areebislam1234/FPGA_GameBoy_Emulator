// tb_cpu_control.v
// Icarus Verilog testbench for cpu_control, focused on the new jump support
// (JR, JP imm16, JP (HL)). Each jump type gets its own isolated
// reset-and-run phase with a small, deliberately loop-free program --
// combining all of them into one continuous address space turned out to
// risk accidental infinite loops (a backward jump's landing zone falling
// through right back into the jump that caused it), so a clean reset
// between phases sidesteps that entirely.
//
// Regression coverage for Blocks 0/1/2 and memory writes already lives in
// the existing tb_cpu_control.v history (register file / ALU / LD /
// memory-write tests) -- this file focuses specifically on control flow,
// since none of that existing decode logic was touched by this change.

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

    always @(*) begin
        mem_data_in = mem[mem_addr];
    end

    always @(posedge clk) begin
        if (mem_write_en) begin
            mem[mem_addr] <= mem_write_data;
        end
    end

    always #5 clk = ~clk;

    // Clears memory, pulses reset, waits for the program (which must end
    // in HALT) to finish, then checks A and unimplemented.
    task run_phase(input [16*8-1:0] name, input [7:0] exp_a, input exp_unimpl);
        begin
            rst = 1;
            repeat (3) @(negedge clk);
            rst = 0;

            // Generous cycle budget -- these programs are short (a jump,
            // one landing instruction, HALT), so this comfortably covers
            // even the 5-cycle JP imm16 path with room to spare.
            repeat (30) @(posedge clk);
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
        $dumpfile("cpu_control.vcd");
        $dumpvars(0, tb_cpu_control);

        // ---- Phase 1: JR forward (+offset) ----
        // JR +3 skips over a "poison" LD A,d8=0xFF that should never
        // execute; landing zone sets A=0x11 to confirm the jump worked.
        mem[0] = 8'h18; mem[1] = 8'h03;           // JR +3 -> target = 2+3 = 5
        mem[2] = 8'h3E; mem[3] = 8'hFF;           // poison LD A,d8 (skipped)
        mem[4] = 8'h00;                            // padding
        mem[5] = 8'h3E; mem[6] = 8'h11;           // landing zone: LD A,d8=0x11
        mem[7] = 8'h76;                            // HALT
        run_phase("JR forward", 8'h11, 1'b0);

        // ---- Phase 2: JR backward (-offset) ----
        // A lead-in JR jumps forward to address 10 (skipping the real
        // backward-landing zone at address 3, keeping it untouched on the
        // first pass). From address 10, a second JR with a negative
        // offset jumps BACK to address 3, which immediately halts after
        // setting A -- preventing any risk of looping back through either JR.
        mem[0]  = 8'h18; mem[1] = 8'h08;          // JR +8 -> target = 2+8 = 10 (lead-in)
        mem[2]  = 8'h00;                           // padding (skipped)
        mem[3]  = 8'h3E; mem[4] = 8'h22;          // backward-landing zone: LD A,d8=0x22
        mem[5]  = 8'h76;                           // HALT -- stops here, no fallthrough risk
        mem[6]  = 8'h00; mem[7] = 8'h00; mem[8] = 8'h00; mem[9] = 8'h00; // padding
        mem[10] = 8'h18; mem[11] = 8'hF7;          // JR -9 -> target = 12-9 = 3
        run_phase("JR backward", 8'h22, 1'b0);

        // ---- Phase 3: JP imm16 (absolute) ----
        mem[0] = 8'hC3; mem[1] = 8'h20; mem[2] = 8'h00; // JP 0x0020
        mem[16'h0020] = 8'h3E; mem[16'h0021] = 8'h33;    // landing zone: LD A,d8=0x33
        mem[16'h0022] = 8'h76;                            // HALT
        run_phase("JP imm16", 8'h33, 1'b0);

        // ---- Phase 4: JP (HL) ----
        mem[0] = 8'hE9;                                    // JP (HL)
        mem[16'h0040] = 8'h3E; mem[16'h0041] = 8'h44;      // landing zone: LD A,d8=0x44
        mem[16'h0042] = 8'h76;                              // HALT

        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        // Preset H:L to point at the landing zone -- same hierarchical
        // technique used throughout this project, since LD r16,imm16
        // isn't built yet.
        dut.rf.h_reg = 8'h00;
        dut.rf.l_reg = 8'h40;

        repeat (30) @(posedge clk);
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
