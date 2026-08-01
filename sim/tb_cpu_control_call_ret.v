// tb_cpu_control_call_ret.v
// Icarus Verilog testbench for cpu_control's CALL/RET support.
//
// The real test here isn't CALL or RET in isolation -- it's a full
// round trip: CALL jumps into a subroutine, the subroutine runs and sets
// a distinguishing register value, RET returns, and execution correctly
// resumes at the instruction right after the original CALL (not before
// it, not somewhere else) -- proving the return address was computed,
// pushed, and popped correctly.

`timescale 1ns/1ps

module tb_cpu_control_call_ret;

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

    initial begin
        $dumpfile("cpu_control_call_ret.vcd");
        $dumpvars(0, tb_cpu_control_call_ret);

        // Main program
        mem[0] = 8'hCD; mem[1] = 8'h10; mem[2] = 8'h00; // CALL 0x0010
        mem[3] = 8'h3E; mem[4] = 8'h99;                  // LD A,d8=0x99 (runs only if RET lands exactly here)
        mem[5] = 8'h76;                                   // HALT

        // Subroutine at 0x0010
        mem[16'h0010] = 8'h06; mem[16'h0011] = 8'h42;     // LD B,d8=0x42 (confirms we entered)
        mem[16'h0012] = 8'hC9;                             // RET

        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;

        dut.rf.sp_reg = 16'h8000; // preset SP, well clear of any real data

        // CALL takes 7 cycles (FETCH/DECODE/MEM_READ/MEM_READ2/EXECUTE/
        // CALL_PUSH_HI/CALL_PUSH_LO)
        repeat (7) @(posedge clk);
        #1;
        if (pc !== 16'h0010) begin
            $display("FAIL: after CALL -- pc expected 0010 got %04h", pc);
            errors = errors + 1;
        end
        if (mem[16'h7FFF] !== 8'h00 || mem[16'h7FFE] !== 8'h03) begin
            $display("FAIL: CALL -- return address on stack expected 0003, got hi=%02h lo=%02h",
                      mem[16'h7FFF], mem[16'h7FFE]);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h7FFE) begin
            $display("FAIL: CALL -- SP expected 7FFE got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end
        if (unimplemented !== 1'b0) begin
            $display("FAIL: CALL -- unimplemented expected 0 got %b", unimplemented);
            errors = errors + 1;
        end

        // LD B,d8 inside the subroutine -- 4 cycles
        repeat (4) @(posedge clk);
        #1;
        if (dut.rf.b_reg !== 8'h42) begin
            $display("FAIL: subroutine LD B,d8 -- B expected 42 got %02h", dut.rf.b_reg);
            errors = errors + 1;
        end

        // RET -- 5 cycles (FETCH/DECODE/MEM_READ/RET_LO/RET_HI)
        repeat (5) @(posedge clk);
        #1;
        if (pc !== 16'h0003) begin
            $display("FAIL: after RET -- pc expected 0003 (right after the CALL) got %04h", pc);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h8000) begin
            $display("FAIL: after RET -- SP expected back to 8000 got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        // LD A,d8 -- only executes correctly if RET truly landed at 0x0003
        repeat (4) @(posedge clk);
        #1;
        if (dut.rf.a_reg !== 8'h99) begin
            $display("FAIL: post-RET LD A,d8 -- A expected 99 got %02h (execution did not resume correctly)",
                      dut.rf.a_reg);
            errors = errors + 1;
        end

        // HALT
        repeat (4) @(posedge clk);
        #1;
        if (halted !== 1'b1) begin
            $display("FAIL: expected halted=1, got %b", halted);
            errors = errors + 1;
        end
        if (dut.rf.b_reg !== 8'h42) begin
            $display("FAIL: final check -- B should still be 42, got %02h", dut.rf.b_reg);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASSED: all CALL/RET checks correct");
        else
            $display("FAILED: %0d error(s) found", errors);

        $finish;
    end

endmodule
