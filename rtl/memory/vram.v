module vram(

input clk,

input write_enable,

input [12:0] address,

input [7:0] write_data,

output reg [7:0] read_data

);

reg [7:0] memory [0:8191];

always @(posedge clk)

begin

    if(write_enable)

        memory[address] <= write_data;

    read_data <= memory[address];

end

endmodule