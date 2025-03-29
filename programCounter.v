module programCounter (clk, rst, PCin, PCout,en);
	
	//inputs
	input clk, rst,en;
	input [31:0] PCin;
	reg [1:0]counter;
	//outputs 
	output reg [31:0] PCout;
	
	//Counter logic
	always@(posedge clk, negedge rst) begin
		if(~rst) begin
			PCout <= 0;
			counter=0;
		end
		else begin
			if(~en) begin
				PCout <= PCin;
			end
		end
	end
	
endmodule
