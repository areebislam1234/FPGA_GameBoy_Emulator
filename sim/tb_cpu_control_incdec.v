// tb_cpu_control_incdec.v
// Icarus Verilog testbench for cpu_control's INC/DEC r8 and INC/DEC r16.
//
// The carry-preservation checks are deliberately constructed so a bug
// CAN'T hide by coincidence: each case is picked so the underlying
// ADD/SUB math would naturally produce a DIFFERENT carry value than the
// one preset -- so if preservation were broken, these tests would catch
// it. (A test where the "correct" and "naturally computed" carry happen
// to match wouldn't actually prove anything.)

`timescale 1ns/1ps

module tb_cpu_control_incdec;

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

    task wait4; begin repeat (4) @(posedge clk); #1; end endtask
    task wait5; begin repeat (5) @(posedge clk); #1; end endtask

    initial begin
        $dumpfile("cpu_control_incdec.vcd");
        $dumpvars(0, tb_cpu_control_incdec);

        mem[0] = 8'h3C; // INC A
        mem[1] = 8'h05; // DEC B
        mem[2] = 8'h0C; // INC C
        mem[3] = 8'h03; // INC BC
        mem[4] = 8'h1B; // DEC DE
        mem[5] = 8'h23; // INC HL
        mem[6] = 8'h33; // INC SP
        mem[7] = 8'h3B; // DEC SP
        mem[8] = 8'h76; // HALT

        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;

        dut.rf.a_reg  = 8'hFF;
        dut.rf.b_reg  = 8'h00;
        dut.rf.c_reg  = 8'h05;
        dut.rf.d_reg  = 8'h00;
        dut.rf.e_reg  = 8'h01;
        dut.rf.h_reg  = 8'hFF;
        dut.rf.l_reg  = 8'hFF;
        dut.rf.sp_reg = 16'h1234;
        dut.rf.f_reg  = 4'b0000; // Z=0 N=0 H=0 C=0

        // INC A: 0xFF -> 0x00. Real ADD math would set C=1 (bit8 overflow).
        // Preset C=0 -- if it reads back 0, that's genuine preservation,
        // not a coincidence.
        wait4;
        if (dut.rf.a_reg !== 8'h00) begin
            $display("FAIL: INC A -- expected 00 got %02h", dut.rf.a_reg);
            errors = errors + 1;
        end
        if (dut.rf.f_reg !== 4'b1010) begin // Z=1 N=0 H=1 C=0(preserved)
            $display("FAIL: INC A -- flags expected 1010 got %04b", dut.rf.f_reg);
            errors = errors + 1;
        end
        if (unimplemented !== 1'b0) begin
            $display("FAIL: INC A -- unimplemented expected 0 got %b", unimplemented);
            errors = errors + 1;
        end

        // DEC B: 0x00 -> 0xFF. Real SUB math would set C=1 (borrow).
        // C is still 0 from before -- confirms preservation in the SUB direction too.
        wait4;
        if (dut.rf.b_reg !== 8'hFF) begin
            $display("FAIL: DEC B -- expected FF got %02h", dut.rf.b_reg);
            errors = errors + 1;
        end
        if (dut.rf.f_reg !== 4'b0110) begin // Z=0 N=1 H=1 C=0(preserved)
            $display("FAIL: DEC B -- flags expected 0110 got %04b", dut.rf.f_reg);
            errors = errors + 1;
        end

        // Flip carry to 1, then INC something where the real ADD math would
        // NOT set carry (no bit7 overflow) -- proves preservation the
        // opposite direction: staying 1 when a fresh computation would give 0.
        dut.rf.f_reg[0] = 1'b1;
        wait4;
        if (dut.rf.c_reg !== 8'h06) begin
            $display("FAIL: INC C -- expected 06 got %02h", dut.rf.c_reg);
            errors = errors + 1;
        end
        if (dut.rf.f_reg !== 4'b0001) begin // Z=0 N=0 H=0 C=1(preserved)
            $display("FAIL: INC C -- flags expected 0001 got %04b", dut.rf.f_reg);
            errors = errors + 1;
        end

        // INC BC: B=0xFF C=0x06 (from above) -> BC=0xFF06 -> 0xFF07.
        // 16-bit INC/DEC touch NO flags -- confirm they stay exactly as
        // INC C left them.
        wait5;
        if (dut.rf.b_reg !== 8'hFF || dut.rf.c_reg !== 8'h07) begin
            $display("FAIL: INC BC -- expected B=FF C=07, got B=%02h C=%02h",
                      dut.rf.b_reg, dut.rf.c_reg);
            errors = errors + 1;
        end
        if (dut.rf.f_reg !== 4'b0001) begin
            $display("FAIL: INC BC -- flags should be untouched, expected 0001 got %04b", dut.rf.f_reg);
            errors = errors + 1;
        end

        // DEC DE: D=0x00 E=0x01 -> DE=0x0001 -> 0x0000
        wait5;
        if (dut.rf.d_reg !== 8'h00 || dut.rf.e_reg !== 8'h00) begin
            $display("FAIL: DEC DE -- expected D=00 E=00, got D=%02h E=%02h",
                      dut.rf.d_reg, dut.rf.e_reg);
            errors = errors + 1;
        end

        // INC HL: HL=0xFFFF -> wraps to 0x0000 (no explicit modulo needed,
        // just natural 16-bit unsigned wraparound)
        wait5;
        if (dut.rf.h_reg !== 8'h00 || dut.rf.l_reg !== 8'h00) begin
            $display("FAIL: INC HL wraparound -- expected H=00 L=00, got H=%02h L=%02h",
                      dut.rf.h_reg, dut.rf.l_reg);
            errors = errors + 1;
        end

        // INC SP: 0x1234 -> 0x1235 (single write, no REG_WRITE_LOW detour)
        wait4;
        if (dut.rf.sp_reg !== 16'h1235) begin
            $display("FAIL: INC SP -- expected 1235 got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        // DEC SP: 0x1235 -> 0x1234
        wait4;
        if (dut.rf.sp_reg !== 16'h1234) begin
            $display("FAIL: DEC SP -- expected 1234 got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        // HALT
        wait4;
        if (halted !== 1'b1) begin
            $display("FAIL: expected halted=1, got %b", halted);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASSED: all INC/DEC checks correct");
        else
            $display("FAILED: %0d error(s) found", errors);

        $finish;
    end

endmodule
