`include "adder.v"
module addTB;
    reg [5:0]in1;
    reg [5:0]in2;
    wire [5:0] out;


    adder add(in1, in2, out);
    integer i;
    initial begin
        i=0;
        in1=0;
        in2=i;
        #4;
        in1=3;
        #4;
        in1=2;
    end
    always #2  begin 
        i=i+1;
        in2=i;

    end
endmodule