module compLogic(op1,op2,compIN/* rt */,compOUT,one,zero);
    input  [31:0] op1,op2;
    input compIN;
    output reg compOUT,one,zero;

    always @ (*) begin 
		
		if(op1==op2) begin
			zero=1;
			one=0;
		end
		else begin
			zero=0;
			one=1;
		end
	
	end

    always @(compIN) begin
		case(compIN)
			0: begin
				if(operand1<0) begin
					compOUT=1;
				end
				else begin
					compOUT=0;
				end
			end
			1: begin
				if(operand1>=0) begin
					compOUT=1;
				end
				else begin
					compOUT=0;
				end
			end

		endcase
	end



endmodule