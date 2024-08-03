`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* Decoder Module & Ports Declaration*/
module asrv32_decoder
    (
        /* Inputs */
        input wire i_clk, i_rst_n,      // Added clock and reset to register the outputs
        input wire[31:0] i_inst,        // The 32-bit instruction to be decoded.
        
        /* Outputs */
        // o_rs1_addr and o_rs2_addr are already registered in the basereg module, hence wire datatype here in decoder definition
        output wire[4:0] o_rs1_addr,    // Address for register source 1. 
        output wire[4:0] o_rs2_addr,    // Address for register source 2.
        // Leaving o_rd_addr as is for now..
        output wire[4:0] o_rd_addr,     // Address for the destination register.
        // Need to hold value for later decoding, hence changed from wire to reg type
        output reg[31:0] o_imm,         // The sign-extended immediate value extracted from the instruction.
        output reg[2:0] o_funct3,       // Function type (3-bit field from the instruction). 
        output reg[`OPCODE_WIDTH-1:0] o_opcode, // Opcode type of the instruction.
        output reg[`ALU_WIDTH-1:0] o_alu_op,    // ALU operation type.

    );

/* Internal Signals and Registers to hold input values for decoding */

    /* Opcode Types */
    reg opcode_rtype_d;
    reg opcode_itype_d;
    reg opcode_load_d;
    reg opcode_store_d;
    reg opcode_branch_d;
    reg opcode_jal_d;
    reg opcode_jalr_d;
    reg opcode_lui_d;
    reg opcode_auipc_d;
    reg opcode_system_d;
    reg opcode_fence_d;
    /* ALU Operations */
    reg alu_add_d;
    reg alu_sub_d;
    reg alu_slt_d;
    reg alu_sltu_d;
    reg alu_xor_d;
    reg alu_or_d;
    reg alu_and_d;
    reg alu_sll_d;
    reg alu_srl_d;
    reg alu_sra_d;
    reg alu_eq_d; 
    reg alu_neq_d;
    reg alu_ge_d; 
    reg alu_geu_d;

/* Functionality Outline: */

/* Operand Address, Opcode & Function Type Extraction: 
The addresses for the source registers (rs1, rs2) and the destination register (rd) are extracted from the instruction.
These addresses are used to read the values from the register file in the next stage.
*/

    //o_rs1_addr and o_rs2_addr are not registered since rv32i_basereg module do the registering itself
    assign o_rs1_addr = i_inst[19:15]; 
    assign o_rs2_addr = i_inst[24:20];  
    assign o_rd_addr = i_inst[11:7];

    wire[2:0] funct3_d = i_inst[14:12];
    wire[6:0] opcode = i_inst[6:0];

    reg[31:0] imm_d;

/* Opcode Type Decoding:
Identify the type of instruction based on its opcode.
These decoded signals control the flow of data and determine the operations required in the next stages.
*/

/* ALU Operation Decoding:
Based on the opcode and function fields (funct3), the required ALU operation is decoded.
Control signals are set to indicate the desired ALU operation to be performed in the execution stage.

/* Immediate Value Extraction:
Depending on the instruction type, the immediate value is extracted and sign-extended. This immediate value can be used in arithmetic operations, load/store instructions, or as an offset for branches and jumps.
*/