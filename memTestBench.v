`include "dataMemory.v"
module tb;
    /*
        input	[7:0]  address;
	input	  clock;
	input	[31:0]  data;
	input	  rden;
	input	  wren;
	output	[31:0]  q;
    */
    reg [7:0] address;
    reg clk;
    reg [31:0] data;
    reg wren,rden;
    wire [31:0] q;

    dataMemory DM(address,clk,data,rden,wren,q);

    initial begin
        clk=0;
        address=0;
        data=1;
        wren=1;
        rden=0;
        #20;
        address=0;
        data=1;
        wren=0;
        rden=1;
        #20;
        address=1;
        data=2;
        wren=0;
        rden=1;
        #20;
        address=1;
        data=2;
        wren=1;
        rden=0;
        #20;
        address=1;
        data=3;
        wren=0;
        rden=1;

    end
    always #10 clk=~clk;
endmodule