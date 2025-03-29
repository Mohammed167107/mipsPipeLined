module processor(clk, rst, PC);
	reg slowerClk;
	//inputs
	input clk, rst;
	reg s;
	parameter second=500000000;
	//outputs
	output [31:0] PC;
	
	wire [31:0] instruction, writeData, readData1, readData2, extImm, ALUin2, ALUResult, memoryReadData,shifted,next,PCPlus1,nextPC,adderResult,shiftedAdd;
	wire [15:0] imm;
	wire [5:0] opCode, funct;
	wire [31:0] nextPC1,nextPC2; ///nextPC1 jump pcr
	wire [4:0] rs, rt, rd, writeRegister,shift;
	wire [3:0] ALUOp;
	wire [1:0] MemtoReg;
	wire  Branch, MemReadEn, MemWriteEn, RegWriteEn, ALUSrc, zero, pcsrc,jump,pcr,brslct,one,zero1,compIN,compOUT,compsig,sel;
	wire [1:0] RegDst;
	
	assign opCode = instruction[31:26];
	assign rs = instruction[25:21];
	assign rt = instruction[20:16];
	assign rd = instruction[15:11];
	assign imm = instruction[15:0];
	assign funct = instruction[5:0];
	assign shift= instruction[10:6];

	/// Forwarding unit wires
	wire [4:0] ID_EX_Rs, ID_EX_Rt, EX_MEM_Rd, MEM_WB_Rd;
	wire EX_MEM_RegWrite,MEM_WB_RegWrite;
	wire [1:0] ForwardA,ForwardB;
	wire [31:0] fwAOut,fwBOut;

	///// Pipelines wires ...
	wire [63:0] iF;
	wire [224:0] iD;
	wire [105:0] eX;
	wire [103:0] mem;
	wire flush,en,enO,enO2;
	mux2x1 #(32) PCMux(.in1(iD[224:193] /*PCPlus1*/), .in2(iD[46:15]/*adderResult*/), .s(PCsrc), .out(nextPC));
	mux2x1 #(32) pcjump(.in1(iD[78:47]/*shifted*/),.in2(iD[46:15] /*readData1*/),.s(iD[179]/*jump*/),.out(next));
	mux2x1 #(32)pcMux(.in1(nextPC),.in2(next),.s(iD[178] /*pcr*/),.out(nextPC1)); /// this mux added for jr instruction
	mux2x1 bmux(.in1(nextPC1),.in2(iD[46:15]/*adderResult*/),.s(sel),.out(nextPC2));
	
	///Dflipflop flop(.in(en),.out(enO),.clk(clk));
	///Dflipflop flop2(.in(enO),.out(enO2),.clk(clk));
	wire en_for_2cycles;
	TwoCyclePulse en_generator (
	    .clk(clk),
	    .rst(rst),
	    .trigger(en),
	    .enOut(en_for_2cycles)
	);
	programCounter pc(.clk(clk), .rst(rst), .PCin(nextPC2), .PCout(PC),.en(en_for_2cycles));
	
	adder PCAdder(.in1(PC), .in2(32'd4), .out(PCPlus1));	
	
	instructionMemory IM(.address(PC), .clock(clk), .q(instruction));
	
	IFID ifid(.out(iF),.in({PCPlus1,instruction}),.clk(clk),.en(1'b1),.rst(rst),.flush(flush));

	////////////////////////// Fetching stage


	controlUnit CU(.opCode(iF[31:26]), .funct(iF[5:0]), 
				      .RegDst(RegDst), .Branch(Branch), .MemReadEn(MemReadEn), .MemtoReg(MemtoReg),
				      .ALUOp(ALUOp), .MemWriteEn(MemWriteEn), .RegWriteEn(RegWriteEn), .ALUSrc(ALUSrc),.jump(jump),.pcr(pcr),.pcsrc(pcsrc),.brslct(brslct),.compsig(compsig));
	
	
	
	registerFile RF(.clk(clk), .rst(rst), .we(mem[101] /*RegWriteEn*/), 
					    .readRegister1(iF[25:21]), .readRegister2(iF[20:16]), .writeRegister(mem[4:0] /*writeRegister*/),
					    .writeData(writeData/*from the mux WBmux*/), .readData1(readData1), .readData2(readData2));
						 
	SignExtender SignExtend(.in(iF[15:0] /* imm */), .out(extImm));   
	shifter #(32) addshif (.in({{16{iF[15]}},iF[15:0]} /* imm */),.out(shiftedAdd));
	shifter #(26) shif(.in(iF[25:0] /* instruction[25:0] */),.out(shifted));
	adder branchAdder(.in1(iF[63:32] /* PCPlus*/), .in2(shiftedAdd), .out(adderResult));
	IDEX idex(.out(iD),.in({iF[63:32],RegDst, Branch, MemReadEn, MemtoReg, ALUOp, MemWriteEn, RegWriteEn, ALUSrc,jump,pcr,pcsrc,brslct,compsig,readData1,readData2,extImm,shifted,adderResult,iF[25:11]}),.clk(clk),.en(1'b1),.rst(rst),.flush(flush));
	////////////////////////////////////////////////////////////////
	
	/////Execution stage

	forwardingUnit fwu (.IDEX_Rs(iD[14:10]),.IDEX_Rt(iD[9:5]),.EXMEM_Rd(eX[4:0]),.MEMWB_Rd(mem[4:0]),.EXMEM_RegWrite(eX[101]),.MEMWB_RegWrite(mem[103]),.ForwardA(ForwardA),.ForwardB(ForwardB));
	mux3x1 #(32) fwA(.in1(iD[174:143])/* rd1 */, .in2(eX[68:37]) /* ALUR EX */,.in3(writeData), .s(ForwardA), .out(fwAOut));
	mux3x1 #(32) fwB(.in1(iD[142:111])/* rd2 */, .in2(eX[68:37])/*rd*/,.in3(writeData), .s(ForwardB), .out(fwBOut));
	
	mux3x1 #(5) RFMux(.in1(iD[9:5]/*rt*/), .in2(iD[4:0]/*rd*/),.in3(5'd31), .s(iD[192:191] /*RegDst*/), .out(writeRegister /* will be outputted thro the pipes to the RF*/));
	mux2x1 #(32) ALUMux(.in1(fwBOut/*.in1(iD[142:111])*//*FWB*/ /*readdata2*/), .in2(iD[110:79])/* extImm*/, .s(  iD[180] /*ALUSrc*/), .out(ALUin2));
	ALU alu(.operand1(fwAOut /*iD[174:143])*/ /*readdata1*/), .operand2(ALUin2), .opSel( iD[186:183] /*ALUOp*/), .result(ALUResult), .zero(zero),.one(one),.shamt(iD[88:84] /*shift*/),.in1(iD[9:5] /* rt */),.compOUT(compOUT));
	
	mux2x1 #(1) branchselector(.in1(zero),.in2(one),.s(iD[176] /*brslct*/),.out(zero1));
	ANDGate branchAnd(.in1(zero1), .in2(iD[190] /*Branch*/), .out(PCsrc));
	ANDGate bzAND(.in1(iD[175] /*compsig*/),.in2(compOUT),.out(sel));

	branchP controlHZD(.sel(sel),.pcsrc(PCsrc),.jump(iD[178]),.flush(flush),.en(en));
	EX exmem(.out(eX),.in({iD[188:187],iD[181],iD[182],iD[189],iD[224:193],ALUResult,iD[142:111],writeRegister}),.clk(clk),.rst(rst),.flush(flush));

	/////////////////////////////////////////////////////////////////
	
	dataMemory DM(.address(eX[68:37] /*ALUResult*/), .clock(~clk), .data(eX[36:5] /*readData2*/), .rden(eX[101] /*MemReadEn*/), .wren(eX[102] /*MemWriteEn*/), .q(memoryReadData));

	MEM memwb(.out(mem),.in({eX[105:104],eX[103],eX[100:69],eX[68:37],memoryReadData,eX[4:0]}),.clk(clk),.rst(clk));
	mux3x1 #(32) WBMux(.in1(mem[68:37] /*ALUResult*/), .in2(mem[36:5]/*memoryReadData*/),.in3(mem[100:69] /*PCPlus1*/), .s(mem[103:102] /*MemtoReg*/), .out(writeData /*to the register file*/));
	
	
	
	
	
	
	
	/*always @(clk) begin
		if(~rst)
			s=0;
		if(s=second) begin
			clk=~slowerClk;
			s=0;
		end
	end*/
endmodule
/////