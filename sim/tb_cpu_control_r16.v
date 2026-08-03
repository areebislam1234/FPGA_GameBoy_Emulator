// tb_cpu_control_r16.v
// Icarus Verilog testbench for cpu_control's LD r16,imm16 support (all
// four variants) plus NOP -- updated for the REGISTERED-read memory
// timing model.

`timescale 1ns/1ps

module tb_cpu_control_r16;

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

    // LD BC/DE/HL,imm16 now takes 9 cycles: FETCH/WAIT/DECODE/MEM_READ/
    // WAIT/MEM_READ2/WAIT/EXECUTE/REG_WRITE_LOW -- two extra settle
    // cycles (low byte, high byte) plus the universal fetch settle.
    task check_pair_load(input [7:0] exp_hi, input [7:0] exp_lo, input [16*8-1:0] name);
        begin
            repeat (9) @(posedge clk);
            #1;
            if (unimplemented !== 1'b0) begin
                $display("FAIL: %0s -- unimplemented expected 0 got %b", name, unimplemented);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("cpu_control_r16.vcd");
        $dumpvars(0, tb_cpu_control_r16);

        mem[0]  = 8'h01; mem[1]  = 8'h34; mem[2]  = 8'h12; // LD BC,0x1234
        mem[3]  = 8'h11; mem[4]  = 8'h78; mem[5]  = 8'h56; // LD DE,0x5678
        mem[6]  = 8'h21; mem[7]  = 8'hBC; mem[8]  = 8'h9A; // LD HL,0x9ABC
        mem[9]  = 8'h31; mem[10] = 8'hF0; mem[11] = 8'hDE; // LD SP,0xDEF0
        mem[12] = 8'h00;                                    // NOP
        mem[13] = 8'h76;                                    // HALT

        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;

        check_pair_load(8'h12, 8'h34, "LD BC,imm16");
        if (dut.rf.b_reg !== 8'h12 || dut.rf.c_reg !== 8'h34) begin
            $display("FAIL: LD BC,imm16 -- expected B=12 C=34, got B=%02h C=%02h",
                      dut.rf.b_reg, dut.rf.c_reg);
            errors = errors + 1;
        end

        check_pair_load(8'h56, 8'h78, "LD DE,imm16");
        if (dut.rf.d_reg !== 8'h56 || dut.rf.e_reg !== 8'h78) begin
            $display("FAIL: LD DE,imm16 -- expected D=56 E=78, got D=%02h E=%02h",
                      dut.rf.d_reg, dut.rf.e_reg);
            errors = errors + 1;
        end

        check_pair_load(8'h9A, 8'hBC, "LD HL,imm16");
        if (dut.rf.h_reg !== 8'h9A || dut.rf.l_reg !== 8'hBC) begin
            $display("FAIL: LD HL,imm16 -- expected H=9A L=BC, got H=%02h L=%02h",
                      dut.rf.h_reg, dut.rf.l_reg);
            errors = errors + 1;
        end

        // LD SP,imm16: 8 cycles (two reads, but no REG_WRITE_LOW -- single write)
        repeat (8) @(posedge clk);
        #1;
        if (unimplemented !== 1'b0) begin
            $display("FAIL: LD SP,imm16 -- unimplemented expected 0 got %b", unimplemented);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'hDEF0) begin
            $display("FAIL: LD SP,imm16 -- expected DEF0 got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        if (dut.rf.b_reg !== 8'h12 || dut.rf.c_reg !== 8'h34 ||
            dut.rf.d_reg !== 8'h56 || dut.rf.e_reg !== 8'h78 ||
            dut.rf.h_reg !== 8'h9A || dut.rf.l_reg !== 8'hBC) begin
            $display("FAIL: register pairs got cross-contaminated after all loads");
            errors = errors + 1;
        end

        // NOP: 5 cycles now (was 4) -- just the universal fetch settle
        repeat (5) @(posedge clk);
        #1;
        if (unimplemented !== 1'b0) begin
            $display("FAIL: NOP -- unimplemented expected 0 got %b (NOP bug not fixed)", unimplemented);
            errors = errors + 1;
        end

        // HALT: 5 cycles now (was 4)
        repeat (5) @(posedge clk);
        #1;
        if (halted !== 1'b1) begin
            $display("FAIL: expected halted=1 after HALT, got %b", halted);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASSED: all LD r16,imm16 / NOP checks correct");
        else
            $display("FAILED: %0d error(s) found", errors);

        $finish;
    end

endmodule
