module comparator(in1,out);
    input [4:0] in1;
    output reg out;

    always @(in1) begin
        if(in1==5'b00001) /// bgez
            out=1;
        else if(in1==5'b00000) /// bltz
            out=0;
    end

endmodule