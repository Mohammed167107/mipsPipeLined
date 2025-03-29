module instructionMemory(
    address,  // Input: Address to fetch the instruction from
    clock,    // Input: Clock signal (not used in this module but included for compatibility)
    q         // Output: The fetched instruction
);
    input [31:0] address;  // 6-bit address input (supports 64 locations)
    input clock;          // Clock signal (unused in this module)
    output reg [31:0] q;  // 32-bit output register to hold the fetched instruction

    // Instruction Memory (IM) is a 256-word memory, each word is 32 bits
    reg [31:0] IM [255:0];

    // Initialize the Instruction Memory by loading instructions from a file
    initial begin
        // Read instructions from the file "instructionMem.txt" into the IM array
        // The file should contain binary representations of the instructions
        $readmemb("instructionMem.txt", IM);
    end

    // Fetch the instruction from memory based on the address
    always @(address) begin
        // Convert the byte address to a word address by dividing by 4 (right-shift by 2)
        // MIPS uses byte addressing, but memory is word-addressable (4 bytes per word)
        q = IM[address >> 2]; 
    end
endmodule