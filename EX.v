module IDEX(out,in,clk,en,rst,flush);
    input [224:0] in;
    input clk,en,rst,flush;
    output [224:0] out;
    reg [224:0] r;
    always @(posedge clk ) begin
        if(rst==0 || flush==1) begin
            r=0;
        end
        else if(en) begin 
            r=in;

        end
    end
    assign out=r;
endmodule
