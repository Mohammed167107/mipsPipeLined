module EX(out,in,clk,rst,flush);
    input [105:0] in;
    input clk,rst,flush;
    output [105:0] out;
    reg [105:0] r;
    always @(posedge clk ) begin
        
        if(rst==0 || flush==1) begin
            r=0;
        end
        else  begin 
            r=in;

        end
    end
    assign out=r;
endmodule
