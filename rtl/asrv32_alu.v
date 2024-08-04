`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* ALU Module & Ports Declaration*/
module asrv32_alu
    (
        /* Inputs */
        input wire i_clk,i_rst_n,           // Input Clock & Active Low Reset
        input wire i_alu_en,                // Enable Signal indicating the ALU stage (Execute stage) is currently active
        input wire[`ALU_WIDTH-1:0] i_alu,   // ALU operation type from previous stage (Decoder)
        input wire[31:0] i_op1,             // Operand 1 : rs1 or pc
        input wire[31:0] i_op2,             // Operand 2 : rs2 or imm

        /* Outputs */
        output reg[31:0] o_alu_result       // Result of arithmetic operation by ALU
    );

    /* Intermediate Register Declaration: */
    reg[31:0] y_d;  // Store ALU Result
    reg[31:0] a;    // Store Operand 1
    reg[31:0] b;    // Store Operand 2

    /* Internal Parallel Wires for less resource utilisation */
    wire alu_add = i_alu[`ADD];
    wire alu_sub = i_alu[`SUB];
    wire alu_slt = i_alu[`SLT];
    wire alu_sltu = i_alu[`SLTU];
    wire alu_xor = i_alu[`XOR];
    wire alu_or = i_alu[`OR];
    wire alu_and = i_alu[`AND];
    wire alu_sll = i_alu[`SLL];
    wire alu_srl = i_alu[`SRL];
    wire alu_sra = i_alu[`SRA];
    wire alu_eq = i_alu[`EQ];
    wire alu_neq = i_alu[`NEQ];
    wire alu_ge = i_alu[`GE];
    wire alu_geu = i_alu[`GEU];

    /* ALU Core Combinational Logic (Blocking Code): */
    always @* begin  
        y_d = 0;                        // Default value of intermediate register y_d
        a = i_op1;
        b = i_op2;

        if(alu_add) y_d = a + b;        // Addition
        if(alu_sub) y_d = a - b;        // Subtraction
        if(alu_slt || alu_sltu) begin   // Set if less than
            y_d = a < b;                // Less than comparison
            if(alu_slt) y_d = (a[31] ^ b[31])? a[31]:y_d;   // Consider sign bit for signed comparison
        end 
        if(alu_xor) y_d = a ^ b;        // Bitwise XOR
        if(alu_or)  y_d = a | b;        // Bitwise OR
        if(alu_and) y_d = a & b;        // Bitwise AND
        if(alu_sll) y_d = a << b[4:0];  // Shift left logical
        if(alu_srl) y_d = a >> b[4:0];  // Shift right logical
        if(alu_sra) y_d = a >>> b[4:0]; // Shift right arithmetic
        if(alu_eq || alu_neq) begin     // Equality check
            y_d = a == b;               // Check if equal
            if(alu_neq) y_d = !y_d;     // Invert result for not equal
        end
        if(alu_ge || alu_geu) begin     // Greater than or equal check
            y_d = a >= b;               // Check if greater than or equal
            if(alu_ge) y_d = (a[31] ^ a[31])? b[31]:y_d;    // Consider sign bit for signed comparison
        end
    end

    /* Register the ALU Output */
    always @(posedge i_clk, negedge i_rst_n) begin
        if(!i_rst_n) o_alu_result <= 0;                           // Reset output to 0 when reset is active (deasserted)
        else o_alu_result <= i_alu_en ? y_d : o_alu_result;     // Update ALU output if ALU stage (Execute stage) is active, else previous result i.e. keep it constant
    end 

endmodule