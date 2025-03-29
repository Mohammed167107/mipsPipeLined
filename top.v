module tb;
    reg clk, rst;
	
	wire [5:0] PC;
	
	initial begin
		clk = 0;
		rst = 0;
		#4 rst = 0;
        #8 rst =1;
		#4000 $stop;
	end
	
	always #5 clk = ~clk;
	
	processor uut(clk, rst, PC);

endmodule