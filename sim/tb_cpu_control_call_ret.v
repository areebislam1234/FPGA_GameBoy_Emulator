// tb_cpu_control_call_ret.v
// Icarus Verilog testbench for cpu_control's CALL/RET support -- updated
// for the REGISTERED-read memory timing model.
//
// CALL now takes 10 cycles (2 reads for the target address, 2 writes for
// the pushed return address, plus the universal fetch settle). RET now
// takes 8 cycles (2 reads for the popped return address).

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

    initial begin
        $dumpfile("cpu_control_call_ret.vcd");
        $dumpvars(0, tb_cpu_control_call_ret);

        mem[0] = 8'hCD; mem[1] = 8'h10; mem[2] = 8'h00; // CALL 0x0010
        mem[3] = 8'h3E; mem[4] = 8'h99;                  // LD A,d8=0x99
        mem[5] = 8'h76;                                   // HALT

        mem[16'h0010] = 8'h06; mem[16'h0011] = 8'h42;     // LD B,d8=0x42
        mem[16'h0012] = 8'hC9;                             // RET

        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;

        dut.rf.sp_reg = 16'h8000;

        // CALL: 10 cycles now (was 7) -- two reads for the target, two
        // writes for the pushed return address
        repeat (10) @(posedge clk);
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

        // LD B,d8 inside the subroutine -- 6 cycles now (was 4)
        repeat (6) @(posedge clk);
        #1;
        if (dut.rf.b_reg !== 8'h42) begin
            $display("FAIL: subroutine LD B,d8 -- B expected 42 got %02h", dut.rf.b_reg);
            errors = errors + 1;
        end

        // RET -- 8 cycles now (was 5)
        repeat (8) @(posedge clk);
        #1;
        if (pc !== 16'h0003) begin
            $display("FAIL: after RET -- pc expected 0003 (right after the CALL) got %04h", pc);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h8000) begin
            $display("FAIL: after RET -- SP expected back to 8000 got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        // LD A,d8 -- 6 cycles now (was 4)
        repeat (6) @(posedge clk);
        #1;
        if (dut.rf.a_reg !== 8'h99) begin
            $display("FAIL: post-RET LD A,d8 -- A expected 99 got %02h (execution did not resume correctly)",
                      dut.rf.a_reg);
            errors = errors + 1;
        end

        // HALT -- 5 cycles now (was 4)
        repeat (5) @(posedge clk);
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
