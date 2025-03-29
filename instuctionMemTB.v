`include "instructionMemory.v"
module instMemtb;
    
    reg [5:0]address;
    reg   clock;
    wire [31:0]q;

    integer i;
    instructionMemory IM (address,clock,q);
    initial begin
        clock=0;
        i=0;
        address=i;
    end
    always #5 clock=~clock;
    always #10 begin 
        i=i+1;
        address=i;
    end
endmodule
