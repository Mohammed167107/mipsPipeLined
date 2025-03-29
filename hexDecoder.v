module hexDecoder(hexIN,hexOut);
    input [3:0] hexIN;
    output reg [6:0] hexOut;

    always @(hexIN) begin
        case(hexIN) 
            4'h0: hexOut=7'b1000000;
            4'h1: hexOut=7'b1111001;
            4'h2: hexOut=7'b0100100;
            4'h3: hexOut=7'b0110000;
            4'h4: hexOut=7'b0011001;
            4'h5: hexOut=7'b0010010;
            4'h6: hexOut=7'b0000010;
            4'h7: hexOut=7'b1111000;
            4'h8: hexOut=7'b0000000;
            4'h9: hexOut=7'b0011000;
            4'ha: hexOut=7'b0001000;
            4'hb: hexOut=7'b0000011;
            4'hc: hexOut=7'b1000110;
            4'hd: hexOut=7'b0100001;
            4'he: hexOut=7'b0000110;
            4'hf: hexOut=7'b0001110;

        endcase
    end 
endmodule