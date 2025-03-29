module controlUnit(opCode, funct,
				   RegDst, Branch, MemReadEn, MemtoReg,
				   ALUOp, MemWriteEn, RegWriteEn, ALUSrc,jump,pcr,pcsrc,brslct,compsig);
				   
		
	// inputs 
	input wire [5:0] opCode, funct;
	
	// outputs (signals)
	output reg  Branch, MemReadEn,  MemWriteEn, RegWriteEn,jump,pcr,pcsrc,brslct; ///added jump pcsrc
	output reg ALUSrc,compsig;
	output reg [1:0] RegDst,MemtoReg;
	output reg [3:0] ALUOp; /// ALUOp was 3 bits now it is 4 bits
	
	// parameters (opCodes/functs)
	parameter _RType = 6'h0, _addi = 6'h8, _lw = 6'h23, _sw = 6'h2b, _beq = 6'h4;
	parameter _add_ = 6'h20, _sub_ = 6'h22, _and_ = 6'h24, _or_ = 6'h25, _slt_ = 6'h2a;
	parameter _sll_=6'h0, _srl_=6'h2, _nor_=6'h27, _xor_=6'h26,_ori_=6'hd,/*_sgt_=6'h*/ _xori_=6'he;
	parameter _bne_=6'h5, _jr_=6'h8, _jal_=6'h3 ,_bltz_=6'h1, _bgez_=6'h1 , _j_=6'h2;
	parameter _nop_=6'h0;
	/* NOR XOR XORI SGT ORI BNE JR JAL SLL SRL BLTZ BGEZ */  /// the new added insturctions 
	always @(*) begin
	
		RegDst = 1'b0; Branch = 1'b0; MemReadEn = 1'b0; MemtoReg = 1'b0;
		MemWriteEn = 1'b0; RegWriteEn = 1'b0; ALUSrc = 1'b0;
		ALUOp = 3'b0;
		
		case(opCode)
			
			_RType : begin
				
				RegDst = 1'b1;
				Branch = 1'b0;
				MemReadEn = 1'b0;
				MemtoReg = 1'b0;
				MemWriteEn = 1'b0;
				RegWriteEn = 1'b1;
				ALUSrc = 1'b0;
				///pcsrc=0;
				jump = 1'b0;
				pcr=0;
				compsig=0;
				case (funct) 
					
					_add_ : begin
						ALUOp = 3'b010;
					end
						
					_sub_ : begin
						ALUOp = 3'b001;
					end
						
					_and_ : begin
						ALUOp = 3'b000;
					end
						
					_or_ : begin
						ALUOp = 3'b011;
					end
						
					_slt_ : begin
						ALUOp = 3'b100;
					end
					_nor_ : begin
						ALUOp=4'b0101;
					end
					_xor_ : begin
						ALUOp=4'b0110;
					end
					_sll_ : begin
						
						ALUOp=4'b1000;
					end
					_srl_ : begin
						
						ALUOp=4'b1001;
					end
					_jr_:  begin
						jump =1;
						pcr=1;
					end
					///_nop_: begin
					///	RegDst = 1'bx;
					///	Branch = 1'bx;
					///	MemReadEn = 1'bx;
					///	MemtoReg = 1'bx;
					///	MemWriteEn = 1'bx;
					///	RegWriteEn = 1'bx;
					///	ALUSrc = 1'bx;
					///	///pcsrc=0;
					///	jump = 1'bx;
					///	pcr=x;
					///	compsig=x;
					///end
					default: ;
				
				endcase
				
			end
			_jal_: begin
				RegDst = 2;
				Branch = 1'b0;
				MemReadEn = 1'b0;
				MemtoReg = 2;
				///ALUOp = 3'b011;
				MemWriteEn = 1'b0;
				RegWriteEn = 1'b1;
				ALUSrc = 1'b1;
				pcsrc=1;
				jump=0;
				pcr=1;
				compsig=0;
			end
			_j_: begin
				RegDst = 2;
				Branch = 1'b0;
				MemReadEn = 1'b0;
				MemtoReg = 2;
				///ALUOp = 3'b011;
				MemWriteEn = 1'b0;
				RegWriteEn = 1'b0;
				ALUSrc = 1'b1;
				pcsrc=1;
				jump=0;
				pcr=1;
				compsig=0;
			end
			_ori_ : begin
				RegDst = 1'b0;
				Branch = 1'b0;
				MemReadEn = 1'b0;
				MemtoReg = 1'b0;
				ALUOp = 3'b011;
				MemWriteEn = 1'b0;
				RegWriteEn = 1'b1;
				ALUSrc = 1'b1;
				pcr=0;
				compsig=0;
			end
			_xori_ : begin
				RegDst = 1'b0;
				Branch = 1'b0;
				MemReadEn = 1'b0;
				MemtoReg = 1'b0;
				ALUOp = 3'b110;
				MemWriteEn = 1'b0;
				RegWriteEn = 1'b1;
				ALUSrc = 1'b1;
				pcr=0;
				jump=0;
				compsig=0;
			end	
			_addi : begin
				RegDst = 1'b0;
				Branch = 1'b0;
				MemReadEn = 1'b0;
				MemtoReg = 1'b0;
				ALUOp = 3'b010;
				MemWriteEn = 1'b0;
				RegWriteEn = 1'b1;
				ALUSrc = 1'b1;	
				pcr=0;	
				jump=0;	
				compsig=0;	
			end
				
			_lw : begin
				RegDst = 1'b0;
				Branch = 1'b0;
				MemReadEn = 1'b1;
				MemtoReg = 1'b1;
				ALUOp = 3'b010;
				MemWriteEn = 1'b0;
				RegWriteEn = 1'b1;		
				ALUSrc = 1'b1;	
				pcr=0;
				compsig=0;	
			end
				
			_sw : begin
				Branch = 1'b0;
				MemReadEn = 1'b0;
				ALUOp = 3'b010;
				MemWriteEn = 1'b1;
				RegWriteEn = 1'b0;
				ALUSrc = 1'b1;	
				pcr=0;
				brslct=0;
				compsig=0;			
			end
				
			_beq : begin
				Branch = 1'b1;
				MemReadEn = 1'b0;
				ALUOp = 3'b001;
				MemWriteEn = 1'b0;
				RegWriteEn = 1'b0;
				ALUSrc = 1'b0;	
				pcr=0;
				brslct=0;
				compsig=0;			
			end
			_bne_: begin
				Branch = 1'b1;
				MemReadEn = 1'b0;
				ALUOp = 3'b001;
				MemWriteEn = 1'b0;
				RegWriteEn = 1'b0;
				ALUSrc = 1'b0;	
				pcr=0;
				brslct=1;
				compsig=0;
			end
			_bltz_: begin
				Branch = 1'b1;
				MemReadEn = 1'b0;
				ALUOp = 3'b001;
				MemWriteEn = 1'b0;
				RegWriteEn = 1'b0;
				ALUSrc = 1'b0;	
				compsig=1;
			end
			_bgez_: begin
				Branch = 1'b1;
				MemReadEn = 1'b0;
				ALUOp = 3'b001;
				MemWriteEn = 1'b0;
				RegWriteEn = 1'b0;
				ALUSrc = 1'b0;
				compsig=1;
				
			end
			default: ;
				
		endcase
			
	end
	
	
endmodule