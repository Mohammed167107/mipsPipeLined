module MEM (out,in,clk,rst);
    input [103:0] in;
    input clk,rst;
    output [103:0] out;
    reg [103:0] r;
    always @(posedge clk ) begin
        if(~rst) begin
            r=0;
        end
        else begin 
            r=in;

        end
    end
    assign out=r;
    
endmodule