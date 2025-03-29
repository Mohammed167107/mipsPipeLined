module Comparator5bit(equal, a, b);
    input [4:0] a, b;
    output equal;
    assign equal = (a == b);
endmodule

module forwardingUnit(
    output [1:0] ForwardA, ForwardB,
    input [4:0] EXMEM_Rd, MEMWB_Rd, IDEX_Rs, IDEX_Rt,
    input EXMEM_RegWrite, MEMWB_RegWrite
);
    wire EXMEM_Rd_EQ_IDEX_Rs, EXMEM_Rd_EQ_IDEX_Rt;
    wire MEMWB_Rd_EQ_IDEX_Rs, MEMWB_Rd_EQ_IDEX_Rt;

    // Comparators for detecting data hazards
    Comparator5bit cmp1(EXMEM_Rd_EQ_IDEX_Rs, EXMEM_Rd, IDEX_Rs);
    Comparator5bit cmp2(EXMEM_Rd_EQ_IDEX_Rt, EXMEM_Rd, IDEX_Rt);
    Comparator5bit cmp3(MEMWB_Rd_EQ_IDEX_Rs, MEMWB_Rd, IDEX_Rs);
    Comparator5bit cmp4(MEMWB_Rd_EQ_IDEX_Rt, MEMWB_Rd, IDEX_Rt);

    // Forwarding logic for source register Rs (ForwardA)
    assign ForwardA[0] = EXMEM_RegWrite && EXMEM_Rd_EQ_IDEX_Rs && (EXMEM_Rd != 0);  // Forward from EX/MEM stage
    assign ForwardA[1] = MEMWB_RegWrite && MEMWB_Rd_EQ_IDEX_Rs && 
                         ((EXMEM_Rd != IDEX_Rs) || (EXMEM_RegWrite == 0)) && (MEMWB_Rd != 0); // Forward from MEM/WB stage

    // Forwarding logic for source register Rt (ForwardB)
    assign ForwardB[0] = EXMEM_RegWrite && EXMEM_Rd_EQ_IDEX_Rt && (EXMEM_Rd != 0);  // Forward from EX/MEM stage
    assign ForwardB[1] = MEMWB_RegWrite && MEMWB_Rd_EQ_IDEX_Rt && 
                         ((EXMEM_Rd != IDEX_Rt) || (EXMEM_RegWrite == 0)) && (MEMWB_Rd != 0); // Forward from MEM/WB stage
endmodule
