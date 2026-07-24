// tb_register_file.v
// Icarus Verilog testbench for register_file.

`timescale 1ns/1ps

module tb_register_file;

    localparam SEL_B = 3'b000, SEL_C = 3'b001, SEL_D = 3'b010, SEL_E = 3'b011,
               SEL_H = 3'b100, SEL_L = 3'b101, SEL_A = 3'b111;

    reg         clk = 0;
    reg         rst = 1;
    reg         write_en = 0;
    reg  [2:0]  write_sel = 0;
    reg  [7:0]  write_data = 0;
    reg  [2:0]  read_sel_a = 0;
    reg  [2:0]  read_sel_b = 0;
    wire [7:0]  read_data_a;
    wire [7:0]  read_data_b;
    reg         flags_write_en = 0;
    reg  [3:0]  flags_in = 0;
    wire [3:0]  flags_out;
    reg         sp_write_en = 0;
    reg  [15:0] sp_write_data = 0;
    wire [15:0] sp;
    wire [15:0] bc, de, hl, af;

    integer errors = 0;

    register_file dut (
        .clk(clk), .rst(rst),
        .write_en(write_en), .write_sel(write_sel), .write_data(write_data),
        .read_sel_a(read_sel_a), .read_sel_b(read_sel_b),
        .read_data_a(read_data_a), .read_data_b(read_data_b),
        .flags_write_en(flags_write_en), .flags_in(flags_in), .flags_out(flags_out),
        .sp_write_en(sp_write_en), .sp_write_data(sp_write_data), .sp(sp),
        .bc(bc), .de(de), .hl(hl), .af(af)
    );

    always #5 clk = ~clk;

    task write_reg(input [2:0] sel, input [7:0] data);
        begin
            @(negedge clk);
            write_en   = 1;
            write_sel  = sel;
            write_data = data;
            @(negedge clk);
            write_en   = 0;
        end
    endtask

    task check_read(input [2:0] sel, input [7:0] expected, input [8*8-1:0] name);
        begin
            read_sel_a = sel;
            #1; // let combinational read settle
            if (read_data_a !== expected) begin
                $display("FAIL: reg %0s expected %02h got %02h", name, expected, read_data_a);
                errors = errors + 1;
            end
        end
    endtask

    initial begin
        $dumpfile("register_file.vcd");
        $dumpvars(0, tb_register_file);

        rst = 1;
        repeat (2) @(negedge clk);
        rst = 0;

        // Write a distinct value to every register, then verify each one
        // independently -- catches any write_sel decode mistakes (e.g. two
        // registers accidentally aliased to the same case branch).
        write_reg(SEL_B, 8'h11);
        write_reg(SEL_C, 8'h22);
        write_reg(SEL_D, 8'h33);
        write_reg(SEL_E, 8'h44);
        write_reg(SEL_H, 8'h55);
        write_reg(SEL_L, 8'h66);
        write_reg(SEL_A, 8'h77);

        check_read(SEL_B, 8'h11, "B");
        check_read(SEL_C, 8'h22, "C");
        check_read(SEL_D, 8'h33, "D");
        check_read(SEL_E, 8'h44, "E");
        check_read(SEL_H, 8'h55, "H");
        check_read(SEL_L, 8'h66, "L");
        check_read(SEL_A, 8'h77, "A");

        // Dual read ports -- read two different registers at once
        read_sel_a = SEL_B;
        read_sel_b = SEL_C;
        #1;
        if (read_data_a !== 8'h11 || read_data_b !== 8'h22) begin
            $display("FAIL: dual-port read got a=%02h b=%02h, expected a=11 b=22",
                      read_data_a, read_data_b);
            errors = errors + 1;
        end

        // Register pairs -- composed from the 8-bit registers above
        if (bc !== 16'h1122) begin
            $display("FAIL: bc expected 1122 got %04h", bc); errors = errors + 1;
        end
        if (de !== 16'h3344) begin
            $display("FAIL: de expected 3344 got %04h", de); errors = errors + 1;
        end
        if (hl !== 16'h5566) begin
            $display("FAIL: hl expected 5566 got %04h", hl); errors = errors + 1;
        end

        // Flags -- independent write path, lower nibble of F always 0
        @(negedge clk);
        flags_write_en = 1;
        flags_in       = 4'b1010; // Z=1 N=0 H=1 C=0
        @(negedge clk);
        flags_write_en = 0;
        #1;
        if (flags_out !== 4'b1010) begin
            $display("FAIL: flags_out expected 1010 got %b", flags_out);
            errors = errors + 1;
        end
        if (af !== 16'h77A0) begin // A=0x77, F upper nibble=1010, lower nibble=0000
            $display("FAIL: af expected 77A0 got %04h", af);
            errors = errors + 1;
        end

        // Stack pointer -- dedicated 16-bit write path
        @(negedge clk);
        sp_write_en   = 1;
        sp_write_data = 16'hFFFE; // SM83 typically inits SP to 0xFFFE
        @(negedge clk);
        sp_write_en   = 0;
        #1;
        if (sp !== 16'hFFFE) begin
            $display("FAIL: sp expected FFFE got %04h", sp);
            errors = errors + 1;
        end

        // Reset clears general registers but not flags/SP (matches how the
        // real always blocks are scoped -- worth confirming that's actually
        // the intended behavior as the design grows)
        @(negedge clk);
        rst = 1;
        @(negedge clk);
        rst = 0;
        #1;
        check_read(SEL_B, 8'h00, "B (post-reset)");

        if (errors == 0)
            $display("PASSED: all register file checks correct");
        else
            $display("FAILED: %0d error(s) found", errors);

        $finish;
    end

endmodule
