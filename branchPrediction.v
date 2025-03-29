module branchP(sel,pcsrc,jump,flush,en);
    input sel,pcsrc,jump;
    output reg flush,en;

    always @(*) begin
        if(sel==1||pcsrc==1||jump==1) begin
            flush=1;
            en=1;
        end
        else begin
            en=0;
            flush=0;
        end
    end

endmodule