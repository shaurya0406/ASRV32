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
    )

    /* Intermediate Register Declaration: */
    reg[31:0] y_d;

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

    /* ALU Core Logic: */
    always @* begin  
        y_d = 0;                            // Default value of intermediate register y_d

        if(alu_add) y_d = i_op1 + i_op2;    // Addition
        if(alu_sub) y_d = i_op1 - i_op2;    // Subtraction
        if(alu_slt || alu_sltu) begin       // Set if less than
            y_d = i_op1 < i_op2;            // Less than comparison
            if(alu_slt) y_d = (i_op1[31] ^ i_op2[31])? i_op1[31]:y_d;   // Consider sign bit for signed comparison
        end 
        if(alu_xor) y_d = i_op1 ^ i_op2;        // Bitwise XOR
        if(alu_or)  y_d = i_op1 | i_op2;        // Bitwise OR
        if(alu_and) y_d = i_op1 & i_op2;        // Bitwise AND
        if(alu_sll) y_d = i_op1 << i_op2[4:0];  // Shift left logical
        if(alu_srl) y_d = i_op1 >> i_op2[4:0];  // Shift right logical
        if(alu_sra) y_d = i_op1 >>> i_op2[4:0]; // Shift right arithmetic
        if(alu_eq || alu_neq) begin             // Equality check
            y_d = i_op1 == i_op2;               // Check if equal
            if(alu_neq) y_d = !y_d;             // Invert result for not equal
        end
        if(alu_ge || alu_geu) begin             // Greater than or equal check
            y_d = i_op1 >= i_op2;               // Check if greater than or equal
            if(alu_ge) y_d = (i_op1[31] ^ i_op1[31])? i_op2[31]:y_d;    // Consider sign bit for signed comparison
        end
    end

    /* Register the ALU Output */
    always @(posedge i_clk, negedge i_rst_n) begin
        if(!rst_n) o_alu_result <= 0;                           // Reset output to 0 when reset is active (deasserted)
        else o_alu_result <= i_alu_en ? y_d : o_alu_result;     // Update ALU output if ALU stage (Execute stage) is active, else previous result i.e. keep it constant
    end 

endmodule