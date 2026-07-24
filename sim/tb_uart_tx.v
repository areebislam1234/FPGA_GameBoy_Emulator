// tb_uart_tx.v
// Icarus Verilog testbench for uart_tx.
// Scales CLK_FREQ/BAUD_RATE down so simulation runs instantly -- this is
// functionally identical to the real 100MHz/9600 config, just a smaller
// CLKS_PER_BIT so we're not waiting on ~10,000 simulated cycles per bit.

`timescale 1ns/1ps

module tb_uart_tx;

    localparam CLK_FREQ     = 1000;
    localparam BAUD_RATE    = 100;
    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE; // = 10
    localparam CLK_PERIOD   = 10; // ns, arbitrary for simulation time

    reg        clk = 0;
    reg        rst = 1;
    reg        tx_start = 0;
    reg [7:0]  tx_data  = 8'h00;
    wire       tx;
    wire       tx_busy;
    wire       tx_done;

    integer errors = 0;

    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) dut (
        .clk(clk),
        .rst(rst),
        .tx_start(tx_start),
        .tx_data(tx_data),
        .tx(tx),
        .tx_busy(tx_busy),
        .tx_done(tx_done)
    );

    always #(CLK_PERIOD/2) clk = ~clk;

    // Sends one byte, then samples the serial line mid-bit at every
    // position in the frame (start / 8 data bits LSB-first / stop).
    task send_and_check(input [7:0] byte_to_send);
        integer i;
        begin
            @(negedge clk);
            tx_data  = byte_to_send;
            tx_start = 1;
            @(negedge clk);
            tx_start = 0;

            wait (tx == 0); // start bit has begun

            #(CLKS_PER_BIT*CLK_PERIOD/2); // sample mid-bit
            if (tx !== 1'b0) begin
                $display("FAIL: start bit not 0 for byte %02h", byte_to_send);
                errors = errors + 1;
            end

            for (i = 0; i < 8; i = i + 1) begin
                #(CLKS_PER_BIT*CLK_PERIOD);
                if (tx !== byte_to_send[i]) begin
                    $display("FAIL: bit %0d mismatch for byte %02h - got %b expected %b",
                              i, byte_to_send, tx, byte_to_send[i]);
                    errors = errors + 1;
                end
            end

            #(CLKS_PER_BIT*CLK_PERIOD);
            if (tx !== 1'b1) begin
                $display("FAIL: stop bit not 1 for byte %02h", byte_to_send);
                errors = errors + 1;
            end

            wait (tx_busy == 0);
            @(negedge clk);
        end
    endtask

    initial begin
        $dumpfile("uart_tx.vcd");
        $dumpvars(0, tb_uart_tx);

        rst = 1;
        repeat (3) @(negedge clk);
        rst = 0;
        @(negedge clk);

        send_and_check(8'h55); // 0101_0101 - alternating bits
        send_and_check(8'hA3); // 1010_0011
        send_and_check(8'h00); // all zeros
        send_and_check(8'hFF); // all ones

        if (errors == 0)
            $display("PASSED: all UART frames correct");
        else
            $display("FAILED: %0d error(s) found", errors);

        $finish;
    end

endmodule
