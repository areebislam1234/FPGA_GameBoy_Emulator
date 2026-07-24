module rom(

    input clk,

    input [15:0] address,

    output reg [7:0] data

);

reg [7:0] memory [0:32767];

initial begin
    $readmemh("boot_rom.hex", memory);
end

always @(posedge clk)

begin

    data <= memory[address];

end

endmodule