module display(register,clk,rst);
    input [31:0] register;
    input clk,rst;
    wire [6:0] hex0,hex1,hex2,hex3,hex4,hex5;
    
    hexDecoder Hex0(.hexIN(register[3:0]),.hexOut(hex0));
    hexDecoder Hex1(.hexIN(register[7:4]),.hexOut(hex1));
    hexDecoder Hex2(.hexIN(register[11:8]),.hexOut(hex2));
    hexDecoder Hex3(.hexIN(register[15:12]),.hexOut(hex3));
    hexDecoder Hex4(.hexIN(register[19:16]),.hexOut(hex4));
    hexDecoder Hex5(.hexIN(register[23:20]),.hexOut(hex5));


endmodule
