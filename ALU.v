module ALU (
    operand1,  // Input: First 32-bit operand
    operand2,  // Input: Second 32-bit operand
    shamt,     // Input: Shift amount (5 bits) for shift operations
    opSel,     // Input: 4-bit operation selector (determines the ALU operation)
    result,    // Output: 32-bit result of the ALU operation
    in1,    // Input: Comparison control signal (used for comparison operations)
    zero,      // Output: Flag indicating if operand1 == operand2
    one,       // Output: Flag indicating if operand1 != operand2
    compOUT    // Output: Comparison result based on compIN
);

    // Parameters for data and selector widths
    parameter data_width = 32;  // Data width for operands and result
    parameter sel_width = 4;    // Selector width for opSel (increased from 3 to 4)

    // Inputs
    input [4:0] in1;                       // Comparison control signal
    input [data_width - 1 : 0] operand2;  // 32-bit operands
    input signed [data_width - 1 : 0] operand1;
    input [sel_width - 1 : 0] opSel;    // 4-bit operation selector
    input [4:0] shamt;                  // 5-bit shift amount for shift operations

    // Outputs
    output reg [data_width - 1 : 0] result;  // 32-bit result of the ALU operation
    output reg zero, one;                    // Flags for equality/inequality
    output reg compOUT;                      // Comparison result based on compIN

    // Operation Parameters (ALU opcodes)
    parameter   _AND  = 'b000,  // Bitwise AND
                _SUB  = 'b001,  // Subtraction
                _ADD  = 'b010,  // Addition
                _OR   = 'b011,  // Bitwise OR
                _SLT  = 'b100,  // Set Less Than (signed comparison)
                _NOR  = 4'b0101,  // Bitwise NOR
                _XOR  = 4'b0110,  // Bitwise XOR
                _SLL  = 4'b1000,  // Shift Left Logical
                _SRL  = 4'b1001;  // Shift Right Logical

    // Perform Operation
    always @ (*) begin
        case(opSel)
            _ADD: result = operand1 + operand2;  // Addition
            _SUB: result = operand1 - operand2;  // Subtraction
            _AND: result = operand1 & operand2;  // Bitwise AND
            _OR : result = operand1 | operand2;  // Bitwise OR
            _NOR: result = ~(operand1 | operand2);  // Bitwise NOR
            _XOR: result = operand1 ^ operand2;  // Bitwise XOR
            _SLT: result = (operand2 < operand1) ? 1 : 0;  // Set Less Than
            _SLL: result = operand1 << shamt;  // Shift Left Logical
            _SRL: result = operand1 >> shamt;  // Shift Right Logical
        endcase
    end

    // Equality/Inequality Check
    always @ (*) begin
        if (operand1 == operand2) begin
            zero = 1;  // Set zero flag if operands are equal
            one = 0;   // Clear one flag
        end
        else begin
            zero = 0;  // Clear zero flag
            one = 1;   // Set one flag if operands are not equal
        end
    end

    // Comparison Logic
    ////always @(*) begin
    ////    case(compIN)
    ////        0: begin  // Check if operand1 is negative
    ////            if (operand1 < 0) begin
    ////                compOUT = 1;  // Set compOUT if operand1 is negative
    ////            end
    ////            else begin
    ////                compOUT = 0;  // Clear compOUT if operand1 is non-negative
    ////            end
    ////        end
    ////        1: begin  // Check if operand1 is non-negative
    ////            if (operand1 >= 0) begin
    ////                compOUT = 1;  // Set compOUT if operand1 is non-negative
    ////            end
    ////            else begin
    ////                compOUT = 0;  // Clear compOUT if operand1 is negative
    ////            end
    ////        end
    ////    endcase
    ////end
    
    always @(*) begin
        if(in1==5'b00001) begin
           if (operand1 >= 0) begin
                    compOUT = 1;  // Set compOUT if operand1 is non-negative
                end
                else begin
                    compOUT = 0;  // Clear compOUT if operand1 is negative
                end 
        end /// bgez
            
        else if(in1==5'b00000) begin
            if (operand1 < 0) begin
                    compOUT = 1;  // Set compOUT if operand1 is negative
                end
                else begin
                    compOUT = 0;  // Clear compOUT if operand1 is non-negative
                end
        end /// bltz
            

    end

endmodule