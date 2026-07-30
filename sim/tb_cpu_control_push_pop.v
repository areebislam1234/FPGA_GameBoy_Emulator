// tb_cpu_control_push_pop.v
// Icarus Verilog testbench for cpu_control's PUSH/POP support.
//
// Pushes BC, DE, HL (in that order), then pops them back in REVERSE
// order (HL, DE, BC) -- standard LIFO stack discipline -- verifying each
// pair round-trips correctly and SP returns to its original value.
// Also confirms PUSH AF / POP AF correctly flag unimplemented, since
// those are deliberately not supported yet.

`timescale 1ns/1ps

module tb_cpu_control_push_pop;

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

    // PUSH and POP both take 5 cycles (FETCH/DECODE/MEM_READ/two more).
    task wait5;
        begin
            repeat (5) @(posedge clk);
            #1;
        end
    endtask

    task check_unimpl(input exp, input [16*8-1:0] name);
        begin
            if (unimplemented !== exp) begin
                $display("FAIL: %0s -- unimplemented expected %b got %b", name, exp, unimplemented);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("cpu_control_push_pop.vcd");
        $dumpvars(0, tb_cpu_control_push_pop);

        mem[0] = 8'hC5; // PUSH BC
        mem[1] = 8'hD5; // PUSH DE
        mem[2] = 8'hE5; // PUSH HL
        mem[3] = 8'hE1; // POP HL
        mem[4] = 8'hD1; // POP DE
        mem[5] = 8'hC1; // POP BC
        mem[6] = 8'hF5; // PUSH AF -- unsupported
        mem[7] = 8'hF1; // POP AF  -- unsupported
        mem[8] = 8'h76; // HALT

        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;

        dut.rf.b_reg  = 8'h11; dut.rf.c_reg = 8'h22; // BC = 0x1122
        dut.rf.d_reg  = 8'h33; dut.rf.e_reg = 8'h44; // DE = 0x3344
        dut.rf.h_reg  = 8'h55; dut.rf.l_reg = 8'h66; // HL = 0x5566
        dut.rf.sp_reg = 16'h8000;

        // PUSH BC: expect mem[0x7FFF]=0x11 (hi), mem[0x7FFE]=0x22 (lo), SP=0x7FFE
        wait5;
        if (mem[16'h7FFF] !== 8'h11 || mem[16'h7FFE] !== 8'h22) begin
            $display("FAIL: PUSH BC -- mem[7FFF]=%02h mem[7FFE]=%02h, expected 11/22",
                      mem[16'h7FFF], mem[16'h7FFE]);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h7FFE) begin
            $display("FAIL: PUSH BC -- SP expected 7FFE got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end
        check_unimpl(1'b0, "PUSH BC");

        // PUSH DE: mem[0x7FFD]=0x33, mem[0x7FFC]=0x44, SP=0x7FFC
        wait5;
        if (mem[16'h7FFD] !== 8'h33 || mem[16'h7FFC] !== 8'h44) begin
            $display("FAIL: PUSH DE -- mem[7FFD]=%02h mem[7FFC]=%02h, expected 33/44",
                      mem[16'h7FFD], mem[16'h7FFC]);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h7FFC) begin
            $display("FAIL: PUSH DE -- SP expected 7FFC got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        // PUSH HL: mem[0x7FFB]=0x55, mem[0x7FFA]=0x66, SP=0x7FFA
        wait5;
        if (mem[16'h7FFB] !== 8'h55 || mem[16'h7FFA] !== 8'h66) begin
            $display("FAIL: PUSH HL -- mem[7FFB]=%02h mem[7FFA]=%02h, expected 55/66",
                      mem[16'h7FFB], mem[16'h7FFA]);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h7FFA) begin
            $display("FAIL: PUSH HL -- SP expected 7FFA got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        // Overwrite the live registers before popping, so a pass only
        // succeeds if POP genuinely restores values from memory --
        // not because the registers happened to already hold them.
        dut.rf.b_reg = 8'h00; dut.rf.c_reg = 8'h00;
        dut.rf.d_reg = 8'h00; dut.rf.e_reg = 8'h00;
        dut.rf.h_reg = 8'h00; dut.rf.l_reg = 8'h00;

        // POP HL: should come back as 0x5566 (pushed last, popped first -- LIFO)
        wait5;
        if (dut.rf.h_reg !== 8'h55 || dut.rf.l_reg !== 8'h66) begin
            $display("FAIL: POP HL -- expected H=55 L=66, got H=%02h L=%02h",
                      dut.rf.h_reg, dut.rf.l_reg);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h7FFC) begin
            $display("FAIL: POP HL -- SP expected 7FFC got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end
        check_unimpl(1'b0, "POP HL");

        // POP DE: should come back as 0x3344
        wait5;
        if (dut.rf.d_reg !== 8'h33 || dut.rf.e_reg !== 8'h44) begin
            $display("FAIL: POP DE -- expected D=33 E=44, got D=%02h E=%02h",
                      dut.rf.d_reg, dut.rf.e_reg);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h7FFE) begin
            $display("FAIL: POP DE -- SP expected 7FFE got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        // POP BC: should come back as 0x1122, and SP should be all the way
        // back to its original 0x8000 -- confirming 3 pushes + 3 pops nets
        // to zero net stack movement.
        wait5;
        if (dut.rf.b_reg !== 8'h11 || dut.rf.c_reg !== 8'h22) begin
            $display("FAIL: POP BC -- expected B=11 C=22, got B=%02h C=%02h",
                      dut.rf.b_reg, dut.rf.c_reg);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h8000) begin
            $display("FAIL: POP BC -- SP expected 8000 (back to original) got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        // PUSH AF / POP AF -- unsupported, 4 cycles each (fall through to
        // normal EXECUTE rather than the 5-cycle push/pop path), SP must
        // stay untouched.
        repeat (4) @(posedge clk);
        #1;
        check_unimpl(1'b1, "PUSH AF (unsupported)");
        if (dut.rf.sp_reg !== 16'h8000) begin
            $display("FAIL: PUSH AF -- SP should be untouched, expected 8000 got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        repeat (4) @(posedge clk);
        #1;
        check_unimpl(1'b1, "POP AF (unsupported)");
        if (dut.rf.sp_reg !== 16'h8000) begin
            $display("FAIL: POP AF -- SP should be untouched, expected 8000 got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        // HALT
        repeat (4) @(posedge clk);
        #1;
        if (halted !== 1'b1) begin
            $display("FAIL: expected halted=1, got %b", halted);
            errors = errors + 1;
        end

        if (errors == 0)
            $display("PASSED: all PUSH/POP checks correct");
        else
            $display("FAILED: %0d error(s) found", errors);

        $finish;
    end

endmodule
