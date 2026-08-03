// tb_cpu_control_push_pop.v
// Icarus Verilog testbench for cpu_control's PUSH/POP support -- updated
// for the REGISTERED-read memory timing model.
//
// PUSH now takes 6 cycles (no memory reads at all -- just the universal
// fetch settle). POP now takes 8 cycles (two reads: low byte, high byte,
// each needing its own settle cycle).

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

    task wait_push; begin repeat (6) @(posedge clk); #1; end endtask
    task wait_pop;  begin repeat (8) @(posedge clk); #1; end endtask

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

        dut.rf.b_reg  = 8'h11; dut.rf.c_reg = 8'h22;
        dut.rf.d_reg  = 8'h33; dut.rf.e_reg = 8'h44;
        dut.rf.h_reg  = 8'h55; dut.rf.l_reg = 8'h66;
        dut.rf.sp_reg = 16'h8000;

        wait_push;
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

        wait_push;
        if (mem[16'h7FFD] !== 8'h33 || mem[16'h7FFC] !== 8'h44) begin
            $display("FAIL: PUSH DE -- mem[7FFD]=%02h mem[7FFC]=%02h, expected 33/44",
                      mem[16'h7FFD], mem[16'h7FFC]);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h7FFC) begin
            $display("FAIL: PUSH DE -- SP expected 7FFC got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        wait_push;
        if (mem[16'h7FFB] !== 8'h55 || mem[16'h7FFA] !== 8'h66) begin
            $display("FAIL: PUSH HL -- mem[7FFB]=%02h mem[7FFA]=%02h, expected 55/66",
                      mem[16'h7FFB], mem[16'h7FFA]);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h7FFA) begin
            $display("FAIL: PUSH HL -- SP expected 7FFA got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        dut.rf.b_reg = 8'h00; dut.rf.c_reg = 8'h00;
        dut.rf.d_reg = 8'h00; dut.rf.e_reg = 8'h00;
        dut.rf.h_reg = 8'h00; dut.rf.l_reg = 8'h00;

        wait_pop;
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

        wait_pop;
        if (dut.rf.d_reg !== 8'h33 || dut.rf.e_reg !== 8'h44) begin
            $display("FAIL: POP DE -- expected D=33 E=44, got D=%02h E=%02h",
                      dut.rf.d_reg, dut.rf.e_reg);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h7FFE) begin
            $display("FAIL: POP DE -- SP expected 7FFE got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        wait_pop;
        if (dut.rf.b_reg !== 8'h11 || dut.rf.c_reg !== 8'h22) begin
            $display("FAIL: POP BC -- expected B=11 C=22, got B=%02h C=%02h",
                      dut.rf.b_reg, dut.rf.c_reg);
            errors = errors + 1;
        end
        if (dut.rf.sp_reg !== 16'h8000) begin
            $display("FAIL: POP BC -- SP expected 8000 (back to original) got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        // PUSH AF / POP AF -- unsupported, 5 cycles each now (fall through
        // to normal EXECUTE with no memory involvement)
        repeat (5) @(posedge clk);
        #1;
        check_unimpl(1'b1, "PUSH AF (unsupported)");
        if (dut.rf.sp_reg !== 16'h8000) begin
            $display("FAIL: PUSH AF -- SP should be untouched, expected 8000 got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        repeat (5) @(posedge clk);
        #1;
        check_unimpl(1'b1, "POP AF (unsupported)");
        if (dut.rf.sp_reg !== 16'h8000) begin
            $display("FAIL: POP AF -- SP should be untouched, expected 8000 got %04h", dut.rf.sp_reg);
            errors = errors + 1;
        end

        // HALT: 5 cycles now (was 4)
        repeat (5) @(posedge clk);
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
