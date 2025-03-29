module Dflipflop(in,out,clk);
    input in,clk;
    output reg out;

    always@(posedge clk) begin
        out<=in;
    end

endmodule