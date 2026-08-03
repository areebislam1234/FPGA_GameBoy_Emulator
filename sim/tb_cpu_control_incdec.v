// tb_cpu_control_incdec.v
// Icarus Verilog testbench for cpu_control's INC/DEC r8 and INC/DEC r16 --
// updated for the REGISTERED-read memory timing model.
//
// None of these instructions touch memory at all (pure register/ALU
// arithmetic), so each only pays the universal +1 fetch-settle cycle:
// register/SP variants go from 4 to 5 cycles, pair variants (needing
// REG_WRITE_LOW) go from 5 to 6.
//
// The carry-preservation checks are unchanged in substance -- deliberately
// constructed so a bug can't hide by coincidence.

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

    task wait_reg;  begin repeat (5) @(posedge clk); #1; end endtask // r8/SP variants
    task wait_pair; begin repeat (6) @(posedge clk); #1; end endtask // BC/DE/HL variants

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
        dut.rf.f_reg  = 4'b0000;

        wait_reg;
        if (dut.rf.a_reg !== 8'h00) begin
            $display("FAIL: INC A -- expected 00 got %02h", dut.rf.a_reg);
            errors = errors + 1;
        end
        if (dut.rf.f_reg !== 4'b1010) begin
            $display("FAIL: INC A -- flags expected 1010 got %04b", dut.rf.f_reg);
            errors = errors + 1;
        end
        if (unimplemented !== 1'b0) begin
            $display("FAIL: INC A -- unimplemented expected 0 got %b", unimplemented);
            errors = errors + 1;
        end

        wait_reg;
        if (dut.rf.b_reg !== 8'hFF) begin
            $display("FAIL: DEC B -- expected FF got %02h", dut.rf.b_reg);
            errors = errors + 1;
        end
        if (dut.rf.f_reg !== 4'b0110) begin
            $display("FAIL: DEC B -- flags expected 0110 got %04b", dut.rf.f_reg);
            errors = errors + 1;
        end

        dut.rf.f_reg[0] = 1'b1;
        wait_reg;
        if (dut.rf.c_reg !== 8'h06) begin
            $display("FAIL: INC C -- expected 06 got %02h", dut.rf.c_reg);
            errors = errors + 1;
        end
        if (dut.rf.f_reg !== 4'b0001) begin
            $display("FAIL: INC C -- flags expected 0001 got %04b", dut.rf.f_reg);
            errors = errors + 1;
        end

        wait_pair;
        if (dut.rf.b_reg !== 8'hFF || dut.rf.c_reg !== 8'h07) begin
            $display("FAIL: INC BC -- expected B=FF C=07, got B=%02h C=%02h",
                      dut.rf.b_reg, dut.rf.c_reg);
            errors = errors + 1;
        end
        if (dut.rf.f_reg !== 4'b0001) begin
            $display("FAIL: INC BC -- flags should be untouched, expected 0001 got %04b", dut.rf.f_reg);
            errors = errors + 1;
        end

        wait_pair;
        if (dut.rf.d_reg !== 8'h00 || dut.rf.e_reg !== 8'h00) begin
            $display("FAIL: DEC DE -- expected D=00 E=00, got D=%02h E=%02h",
                      dut.rf.d_reg, dut.rf.e_reg);
            errors = errors + 1;
        end

        wait_pair;
        if (dut.rf.h_reg !== 8'h00 || dut.rf.l_reg !== 8'h00) begin
            $display("FAIL: INC HL wraparound -- expected H=00 L=00, got H=%02h L=%02h",
                      dut.rf.h_reg, dut.rf.l_reg);
            errors = errors + 1;
        end

        wait_reg;
        if (dut.rf.sp_reg !== 16'h1235) begin
            $display("FAIL: INC SP -- expected 1235 got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        wait_reg;
        if (dut.rf.sp_reg !== 16'h1234) begin
            $display("FAIL: DEC SP -- expected 1234 got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        wait_reg;
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
