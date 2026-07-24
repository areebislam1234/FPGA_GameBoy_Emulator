module address_decoder(

    input  [15:0] address,

    output rom_enable,
    output vram_enable,
    output wram_enable

);

assign rom_enable  = (address <= 16'h7FFF);

assign vram_enable =
    (address >= 16'h8000) &&
    (address <= 16'h9FFF);

assign wram_enable =
    (address >= 16'hC000) &&
    (address <= 16'hDFFF);

endmodule