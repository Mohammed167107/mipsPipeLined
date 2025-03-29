module shifter #(parameter size = 32) (in,out);
    input [size-1:0]in;
    output reg [31:0]out;
    always @(in) begin
        if(size==26)
            out = {{6{in[25]}}, in<<2};
        else if(size==32)
            out=in<<2;
    end
   // always @(in) begin
   //     out =in << 2;
   // end
    ////assign out = {{6{in[25]}}, in};

endmodule