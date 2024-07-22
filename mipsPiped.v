module piped(clk,reset,enable);
    input clk, reset, enable;
    
    wire [31:0] pcOut,PC, nextPC,nextPC1, instruction, PC_plus_4, branch_target, /*jump_target,*/ ALU_result, ALU_input2, mem_data, write_data;
    wire [31:0] read_data1, read_data2, sign_ext_imm, shifted_imm; 
    wire [27:0] shifted_jump;
    wire [4:0] write_reg;
    wire [2:0] alu_op;
    wire [1:0] reg_dst, mem_to_reg ;
    wire reg_write, mem_write, mem_read, pc_src ,jump, branch, zero,alu_src;

    // Pipes outputs...
    wire [63:0] IF;
    wire [153:0] ID;
    wire [105:0] EX;
    wire [103:0] MW;

    // Program Counter
    REG32posclk ProgramCounter(PC, pcOut, clk, reset, enable);

    // Instruction Memory
    Instruction_Memory IM(PC, instruction);
    
    // Adder for PC + 4
    Adder32bit PCAdder(PC_plus_4, PC, 32'd4);
    
    // IFDF Pipe
    IFID ifdf(IF,{PC_plus_4,instruction},clk,reset,enable);

    
    /////////// Decode stage.......


    // Control Unit
    ControlUnit CU(IF[31:26], IF[5:0],alu_op, reg_dst,mem_to_reg,alu_src, reg_write, mem_read,mem_write, branch, jump,pc_src);

    // Shift Left for Jump Address
    ShiftLeft26_by2 JumpShift(shifted_jump, IF[25:0]);   ///28=bits out

    // Register File
    RegFile RF(read_data1, read_data2, IF[25:21], IF[20:16],write_data, MW[4:0], MW[103], clk, reset);
    
    // Sign Extend
    SignExtend SE(sign_ext_imm, IF[15:0]);
    ShiftLeft32_by2 SL2(shifted_imm, sign_ext_imm);

    // Adder for Branch Target
    Adder32bit BranchAdder(branch_target, PC_plus_4, sign_ext_imm/*Later shift Later*/);

    // Comparator for Branch
    Comparator32bit CMP(zero, read_data1, read_data2);
    wire aNd;
    assign aNd= branch && zero;

    IDEX idex(ID,{alu_op, reg_dst,mem_to_reg,alu_src, reg_write, mem_read,mem_write,PC_plus_4,read_data1,read_data2,sign_ext_imm,IF[25:21],IF[20:16],IF[15:11]},clk,reset,enable);
    
    /////////////////////////////////////

    // Mux for ALU input 2
    Mux_2_to_1_32bit ALUMux(ALU_input2, ID[146], ID[46:15], ID[78:47]);

    // ALU
    ALU alu(ID[153:151], ID[110:79], ALU_input2,ALU_result );
    
    // Mux for Write Register
    Mux_3_to_1_5bit WriteRegMux(write_reg, reg_dst,5'd31 , ID[9:5],ID[4:0] );

    EXMEM exmem(EX,{ID[148:147],ID[145:111],ALU_result,ID[78:47],write_reg},clk,reset,enable);

    ///////////////////////////////////////


    // Data Memory
    Data_Memory DM(mem_data, EX[68:37], EX[100:69], EX[101], EX[102], clk); //////now we are here.......
    MEMWB memwb(MW,{EX[103],EX[105:104],EX[100:69],mem_data,EX[68:37],EX[4:0]},clk,reset,enable);

    ///////////////////////////////////////
    Mux_3_to_1_32bit WriteDataMux(write_data, MW[102:101],PC_plus_4 , mem_data,ALU_result);


    Mux_2_to_1_32bit PCBranchMux(nextPC, aNd, branch_target,PC_plus_4);
    Mux_2_to_1_32bit PCJumpMux(nextPC1, jump, {PC_plus_4[31:28], shifted_jump}, read_data1);
    Mux_2_to_1_32bit PCpick(pcOut,pc_src,nextPC1,nextPC);
    // Mux for Write Data
    

endmodule

module ALU(m ,in1,in2,out);
    input [2:0] m;
    input [31:0] in1,in2;
    reg [31:0] o;
    output reg [31:0] out;
    always@(*) begin
        case(m)
            0:out=in1 | in2;
            3'b001:out=in1 & in2;
            2: out=in1^in2;
            3: out=in1+in2;
            4: out= !(in1 | in2);
            5: out=!(in1 & in2);
            6:out= in1<in2 ?1'b1:1'b0;
            7:begin
                o=~in2+1;
                out=in1+o;
            end
        endcase
    end
endmodule

module Data_Memory(
    output reg [31:0] readdata,
    input [31:0] address,
    input [31:0] writedata,
    input memwrite,
    input memread,
    input clk
);
    reg [31:0] DM [255:0]; // 256-word memory

    // Initialize memory with incremental values
    integer i;
    initial begin
        for (i = 0; i < 256; i = i + 1) begin
            DM[i] = i;
        end
    end

    always @(posedge clk) begin
        if (memwrite)
            DM[address >> 2] <= writedata; // Divide by 4 to get the correct word address
    end

    always @(posedge clk) begin
        if (memread)
            readdata <= DM[address >> 2]; // Divide by 4 to get the correct word address
    end
endmodule


module PC (input [31:0] in, input clk, input reset, output  [31:0] out);
reg [31:0] pc;
    always @(posedge clk ) begin
        if(reset)
        begin
        pc<=0;
        end
        else
        begin
        pc<=in;
        end
    end
    assign out=pc;
endmodule

module RegFile(
    readdata1, readdata2, readreg1, readreg2,
    writedata, writereg, regwrite, clk, reset
);
    input regwrite, clk, reset;
    input [4:0] readreg1, readreg2, writereg;
    input [31:0] writedata;
    output [31:0] readdata1, readdata2;

    reg [31:0] registers [31:0];
    integer i;

    always @(posedge clk) begin
        if (!reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                registers[i] <= 32'b0;
            end
        end else if (regwrite) begin
            registers[writereg] = writedata;
        end
    end
    assign readdata1 = registers[readreg1];
    assign readdata2 = registers[readreg2];
endmodule

module REG32posclk (Q, D, clk, reset, enable);
    input clk, reset, enable;
    reg [31:0] pc;
    input [31:0] D;
    output [31:0] Q;

    always @(posedge clk) begin
        if (!reset) begin
            pc <= 32'b0;
        end else if (enable) begin
            pc <= D;
        end
    end
    assign Q=pc;
endmodule

module Instruction_Memory(
    input [31:0] PC,
    output reg [31:0] instruction
);
    reg [31:0] IM [255:0];

    initial begin
        IM[0] = 32'h8C010004;
        IM[1] = 32'h8c02000c;
        IM[2] = 32'h8C030014;
        IM[3] = 32'h8C04001C;
        IM[4] = 32'h00222805;
        IM[5] = 32'h506603ff;
        IM[6] = 32'h00823807;
        IM[7] = 32'h00a44002;
        IM[8] = 32'h44c907ff;
        IM[9] = 32'hAC060008;
        IM[10] = 32'h8c0a0008;
        IM[11] = 32'h00e85800;
        IM[12] = 32'h00246006;
        ///IM[12]=32'hc0c30001;
        ///IM[13]=32'h03e00008;
        ///IM[14]=32'h8ce85800;
        ///IM[15]=32'h01015806;

        // Initialize other locations as needed
    end

    always @(PC) begin
        instruction = IM[PC >> 2]; // Divide by 4 to get the correct word address
    end
endmodule

module IFID(Q,D,clk,reset,enable);
    input clk,reset,enable;
    input [63:0] D;
    output [63:0] Q;
    reg [63:0]q;
    always @(posedge clk) begin
        if(!reset) begin
            q=0;
        end
        else if(enable) begin
            q=D;
        end
    end

    assign Q=q;
endmodule

module IDEX(Q,D,clk,reset,enable);
    input clk,reset,enable;
    input [153:0] D;
    output [153:0] Q;
    reg [153:0] q;
    always @(posedge clk) begin
        if(!reset) begin
            q=0;
        end
        else if(enable) begin
            q=D;
        end
    end
    assign Q=q;
endmodule

module EXMEM(Q,D,clk,reset,enable);
input clk,reset,enable;
    input [105:0] D;
    output [105:0] Q;
    reg [105:0] q;
    always @(posedge clk) begin
        if(!reset) begin
            q=0;
        end
        else if(enable) begin
            q=D;
        end
    end
    assign Q=q;

endmodule

module MEMWB(Q,D,clk,reset,enable);
input clk,reset,enable;
    input [103:0] D;
    output [103:0] Q;
    reg [103:0] q;
    always @(posedge clk) begin
        if(!reset) begin
            q=0;
        end
        else if(enable) begin
            q=D;
        end
    end
    assign Q=q;

endmodule

module shiftLeft(out,in);
    input [31:0] in;
    output  [31:0] out;
    assign out=in>>2;
endmodule
module Adder32bit(out, a, b);
    input [31:0] a, b;
    output [31:0] out;
    assign out = a + b;
endmodule
module SignExtend(out, in);
    input [15:0] in;
    output [31:0] out;
    assign out = {{16{in[15]}}, in};
endmodule
module Comparator32bit(equal, a, b);
    input [31:0] a, b;
    output equal;
    assign equal = (a == b);
endmodule
module ShiftLeft26_by2(out, in);
    input [25:0] in;
    output [27:0] out;
    assign out = {in, 2'b00};
endmodule
module ShiftLeft32_by2(out, in);
    input [31:0] in;
    output [31:0] out;
    assign out = in << 2;
endmodule
module Mux_3_to_1_5bit(out, s, i2, i1, i0);
    input [4:0] i2, i1, i0;
    input [1:0] s;
    output [4:0] out;
    assign  out = (s == 2'b00) ? i0 : (s == 2'b01) ? i1 : i2;
endmodule
module Mux_3_to_1_32bit(out, s, i2, i1, i0);
    input [31:0] i2, i1, i0;
    input [1:0] s;
    output [31:0] out;
    assign  out = (s == 2'b00) ? i0 : (s == 2'b01) ? i1 : i2;
endmodule
module Mux_2_to_1_32bit(out, s, i1, i0);
    input [31:0] i1, i0;
    input s;
    output [31:0] out;
    assign  out = s ? i1 : i0;
endmodule

module tb;
    reg clk, reset, enable;

    piped uut (.clk(clk), .reset(reset), .enable(enable));

    initial begin
        clk = 1;
        reset = 0;
        enable = 1;
        #50 reset = 1;
    end

    always #25 clk = ~clk;
endmodule

module ControlUnit (opcode,func,aluop,regdst,memtoreg,alusrc,regwrite,memread,memwrite,branch,jump,pcsrc);
    input [5:0] opcode,func;
    output reg [2:0] aluop;
    output reg [1:0] regdst,memtoreg;
    output reg alusrc,regwrite,memread,memwrite,branch,jump,pcsrc;
    always @(opcode or func) begin
        if(!opcode) begin
            case(func)
                6'b000000 : begin
                    aluop=3'b000;
                    alusrc=0;
                    regdst=2'b01;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b000001 : begin
                    aluop=3'b001;
                    alusrc=0;
                    regdst=2'b01;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b000010 : begin
                    aluop=3'b010;
                    alusrc=0;
                    regdst=2'b01;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b000011 : begin
                    aluop=3'b011;
                    alusrc=0;
                    regdst=2'b01;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b000100 : begin
                    aluop=3'b100;
                    alusrc=0;
                    regdst=2'b01;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b000101 : begin
                    aluop=3'b101;
                    alusrc=0;
                    regdst=2'b01;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b000110 : begin
                    aluop=3'b110;
                    alusrc=0;
                    regdst=2'b01;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b000111 :begin
                    aluop=3'b111;
                    alusrc=0;
                    regdst=2'b01;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b001000 :begin
                    regwrite=0;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=1;
                end
            endcase
        end
        else begin
            case(opcode)
                6'b010000:begin
                    aluop=3'b000;
                    alusrc=1;
                    regdst=2'b00;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b010001:begin
                    aluop=3'b001;
                    alusrc=1;
                    regdst=2'b00;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b010010:begin
                    aluop=3'b010;
                    alusrc=1;
                    regdst=2'b00;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b010011:begin
                    aluop=3'b011;
                    alusrc=1;
                    regdst=2'b00;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b010100:begin
                    aluop=3'b100;
                    alusrc=1;
                    regdst=2'b00;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b010101:begin
                    aluop=3'b101;
                    alusrc=1;
                    regdst=2'b00;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b010110:begin
                    aluop=3'b110;
                    alusrc=1;
                    regdst=2'b00;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b010111:begin
                    aluop=3'b111;
                    alusrc=1;
                    regdst=2'b00;
                    memtoreg=2'b00;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b100011:begin
                    aluop=3'b011;
                    alusrc=1;
                    regdst=2'b00;
                    memtoreg=2'b01;
                    regwrite=1;
                    memread=1;
                    memwrite=0;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b101011:begin
                    aluop=3'b011;
                    alusrc=1;
                    regwrite=0;
                    memread=0;
                    memwrite=1;
                    branch=0;
                    jump=0;
                    pcsrc=0;
                end
                6'b110000:begin
                    regwrite=0;
                    memread=0;
                    memwrite=0;
                    branch=1;
                    jump=0;
                    pcsrc=0;
                end
                6'b110001:begin
                    regwrite=0;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=1;
                    pcsrc=1;
                end
                6'b110011:begin
                    regdst=2'b10;
                    memtoreg=2'b10;
                    regwrite=1;
                    memread=0;
                    memwrite=0;
                    branch=0;
                    jump=1;
                    pcsrc=1;
                end
            endcase
        end
    end
endmodule