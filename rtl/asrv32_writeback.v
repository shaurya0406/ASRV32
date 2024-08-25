`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* Writeback Module & Ports Declaration*/
module asrv32_writeback #(parameter PC_RESET = 32'h00_00_00_00) (
    
    input wire[2:0] i_funct3,                   // Function type 
    input wire[31:0] i_result_from_alu,         // Output of ALU
    input wire[31:0] i_imm,                     // Immediate value
    input wire[31:0] i_rs1_data,                // Source register 1 value
    input wire[31:0] i_load_data_from_memory,   // Data to be loaded to base register
    input wire[31:0] i_load_data_from_csr,      // CSR value to be loaded to basereg
    input wire[`OPCODE_WIDTH-1:0] i_opcode,     // Opcode
    
    // Trap Handlers
    input wire i_go_to_trap,                    // high before going to trap (if exception/interrupt detected)
    input wire i_return_from_trap,              // high before returning from trap (via MRET)
    input wire[31:0] i_return_address,          // MEPC CSR
    input wire[31:0] i_trap_address,            // MTEVC CSR

    /* Base RegFile Control */
    input wire i_wr_rd_en, //write rd to basereg if enabled (from previous stage)
    output reg o_wr_rd_en, //write rd to the base reg if enabled
    input wire[4:0] i_rd_addr, //address for destination register (from previous stage)
    output reg[4:0] o_rd_addr, //address for destination register
    input wire[31:0] i_rd_data, //value to be written back to destination register (from previous stage)
    output reg[31:0] o_rd_data, //value to be written back to destination register

    /* PC Control */
    input wire[31:0] i_pc, // pc value (from previous stage)
    output reg[31:0] o_next_pc, //new pc value
    output reg o_change_pc, //high if PC needs to jump

    /* Pipeline Control */
    input wire i_ce,    // Global clk enable for pipeline stalling of this stage
    output reg o_stall, // Global stall signal, informs pipeline to stall
    output reg o_flush  // Flush previous stages
);

/* Internal Wires for decoding opcode signals. */
    wire opcode_rtype = i_opcode[`RTYPE];
    wire opcode_itype = i_opcode[`ITYPE];
    wire opcode_load = i_opcode[`LOAD];
    wire opcode_store = i_opcode[`STORE];
    wire opcode_branch = i_opcode[`BRANCH];
    wire opcode_jal = i_opcode[`JAL];
    wire opcode_jalr = i_opcode[`JALR];
    wire opcode_lui = i_opcode[`LUI];
    wire opcode_auipc = i_opcode[`AUIPC];
    wire opcode_system = i_opcode[`SYSTEM];
    wire opcode_fence = i_opcode[`FENCE];


/* Combinational Logic for Writeback Stage */

endmodule