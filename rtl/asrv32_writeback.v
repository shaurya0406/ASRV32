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
    always @(*) begin
        o_stall = 0; // Default: No stall unless conditions require it
        o_flush = 0; // Default: No flush unless a PC change occurs
        o_wr_rd_en = i_wr_rd_en && i_ce && !o_stall; // Enable write to rd if stage is enabled and not stalled
        o_rd_addr = i_rd_addr; // Propagate rd address from input to output
        o_rd = 0; // Default rd value
        o_next_pc = 0; // Default: No change in PC
        o_change_pc = 0; // Default: No PC change unless triggered

        // Check for trap conditions
        if(i_go_to_trap) begin
            o_change_pc = 1; // Change PC if trap condition is met
            o_next_pc = i_trap_address; // Set PC to trap address (mtvec) if interrupt/exception occurs
            o_flush = i_ce; // Flush pipeline stages if this stage is enabled
            o_wr_rd = 0; // Disable writing to rd in trap condition
        end
        
        // Check for return from trap (mret instruction)
        else if(i_return_from_trap) begin
            o_change_pc = 1; // Change PC if returning from trap
            o_next_pc = i_return_address; // Set PC to return address (mepc) if mret is triggered
            o_flush = i_ce; // Flush pipeline stages if this stage is enabled
            o_wr_rd_en = 0; // Disable writing to rd when returning from trap
        end
        
        // Normal operation (no trap or return from trap)
        else begin
            if(i_opcode_load) 
                o_rd = i_data_load; // Load data from memory to rd if LOAD instruction
            else if(i_opcode_system && i_funct3 != 0) begin // CSR write operation
                o_rd = i_csr_out; // Write CSR output to rd if SYSTEM instruction with funct3 non-zero
            end
            else 
                o_rd = i_rd; // Default: Pass ALU-computed rd value
        end    
    end
endmodule