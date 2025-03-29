module mux3x1 #(parameter size = 32) (in1, in2,in3, s, out);

	// inputs	
	input [1:0] s;
	input [size-1:0] in1, in2,in3;
	
	// outputs
	output [size-1:0] out;

	// Unit logic
	    assign out = (s == 2'b00) ? in1 :
                 (s == 2'b01) ? in2 :
                 (s == 2'b10) ? in3 : 
                 {size{1'bx}};
	
endmodule