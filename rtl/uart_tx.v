// uart_tx.v
// Simple 8N1 UART transmitter.
// Frame: 1 start bit (0) -> 8 data bits, LSB first -> 1 stop bit (1)
//
// CLK_FREQ / BAUD_RATE set the bit period in clock cycles. Defaults match
// the Basys3's 100MHz onboard clock at a standard 9600 baud.

module uart_tx #(
    parameter CLK_FREQ  = 100_000_000,
    parameter BAUD_RATE = 9600
)(
    input  wire       clk,
    input  wire       rst,        // synchronous, active-high
    input  wire       tx_start,   // pulse 1 cycle to begin sending tx_data
    input  wire [7:0] tx_data,
    output reg        tx,         // serial line, idles high
    output reg        tx_busy,    // high while a frame is in flight
    output reg        tx_done     // pulses 1 cycle when the frame completes
);

    localparam CLKS_PER_BIT = CLK_FREQ / BAUD_RATE;

    localparam IDLE  = 2'd0,
               START = 2'd1,
               DATA  = 2'd2,
               STOP  = 2'd3;

    reg [1:0]                    state;
    reg [$clog2(CLKS_PER_BIT):0] clk_count;
    reg [2:0]                    bit_index;
    reg [7:0]                    tx_shift;

    always @(posedge clk) begin
        if (rst) begin
            state     <= IDLE;
            tx        <= 1'b1;
            tx_busy   <= 1'b0;
            tx_done   <= 1'b0;
            clk_count <= 0;
            bit_index <= 0;
            tx_shift  <= 8'h00;
        end else begin
            tx_done <= 1'b0; // default; only pulses in STOP below

            case (state)
                IDLE: begin
                    tx        <= 1'b1;
                    clk_count <= 0;
                    bit_index <= 0;
                    if (tx_start) begin
                        tx_busy  <= 1'b1;
                        tx_shift <= tx_data;
                        state    <= START;
                    end else begin
                        tx_busy <= 1'b0;
                    end
                end

                START: begin
                    tx <= 1'b0; // start bit
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 0;
                        state     <= DATA;
                    end
                end

                DATA: begin
                    tx <= tx_shift[bit_index];
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 0;
                        if (bit_index < 3'd7) begin
                            bit_index <= bit_index + 1'b1;
                        end else begin
                            bit_index <= 0;
                            state     <= STOP;
                        end
                    end
                end

                STOP: begin
                    tx <= 1'b1; // stop bit
                    if (clk_count < CLKS_PER_BIT - 1) begin
                        clk_count <= clk_count + 1'b1;
                    end else begin
                        clk_count <= 0;
                        tx_busy   <= 1'b0;
                        tx_done   <= 1'b1;
                        state     <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
