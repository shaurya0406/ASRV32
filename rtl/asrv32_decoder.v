`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* Decoder Module & Ports Declaration*/
module asrv32_decoder
    (
        /* Inputs */
        input wire[31:0] i_inst,        // The 32-bit instruction to be decoded.
        
        /* Outputs */
        output wire[4:0] o_rs1_addr,    // Address for register source 1.
        output wire[4:0] o_rs2_addr,    // Address for register source 2.
        output wire[4:0] o_rd_addr,     // Address for the destination register.
        output reg[31:0] o_imm,         // The sign-extended immediate value extracted from the instruction.
        output wire[2:0] o_funct3,      // Function type (3-bit field from the instruction).
        output wire[`OPCODE_WIDTH-1:0] o_opcode, // Opcode type of the instruction.
        output wire[3:0] o_alu_op,      // ALU operation type.

    );

/* Internal Signals and Registers to hold input values for decoding */

/*
Functionality Outline:

/* Operand Address, Opcode & Function Type Extraction:
The addresses for the source registers (rs1, rs2) and the destination register (rd) are extracted from the instruction.
These addresses are used to read the values from the register file in the next stage.

/* ALU Operation Decoding:
Based on the opcode and functio`n fields (funct3), the required ALU operation is decoded.
Control signals are set to indicate the desired ALU operation to be performed in the execution stage.

/* Immediate Value Extraction:
Depending on the instruction type, the immediate value is extracted and sign-extended. This immediate value can be used in arithmetic operations, load/store instructions, or as an offset for branches and jumps.
*/