`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* Writeback Module & Ports Declaration*/
module asrv32_writeback #(parameter PC_RESET = 32'h00_00_00_00) (
    
    /* Inputs */
    // ! No clock and reset needed as this stage runs parallel to CSR
    // input wire i_clk, i_rst_n,
    // ! No stage enable signal in  pipeline, added pipeline control
    // input wire i_writeback_en,                  // Enable o_wr_rd_en if stage is currently on WRITEBACK Stage
    
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
    initial o_pc = PC_RESET;                    // Initialize PC to PC_RESET

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

// ! Shifted to ALU
// /* Intermediate Registers */
//     reg[31:0] rd_d;     // Intermediate register for destination register data
//     reg[31:0] pc_d;     // Intermediate register for PC
//     reg wr_rd_d;        // Intermediate register for write enable signal
//     reg[31:0] a;        // Intermediate registers for PC calculations
//     wire[31:0] sum;     // Intermediate Signal for ALU calculations 

/* Combinational Logic for Writeback Stage */

    // ! Shifted to ALU
    // assign sum = a + i_imm; // Share adder for all addition operation for less resource utilization (Non Blocking Combinational Logic)   

    //determine next value of PC and Rd
    always @* begin
        rd_d = 0;
        // ! Shifted to Fetch
        pc_d = o_pc + 32'd4; // Default PC increment
        wr_rd_d = 0;
        a = o_pc;
        
        // Trap Handling Logic:
        if(i_go_to_trap) pc_d = i_trap_address;  // Interrupt or exception detected
        else if(i_return_from_trap) pc_d = i_return_address; // Return from trap via mret
        // Normal Operation:
        else begin
            // Set rd_d based on instruction type
            if(opcode_rtype || opcode_itype) rd_d = i_result_from_alu;
            if(opcode_load) rd_d = i_load_data_from_memory;

            // Handle branch instructions
            if(opcode_branch && i_result_from_alu[0]) pc_d = sum; //branch iff value of ALU is 1(true)

            // Handle JAL and JALR instructions
            if (opcode_jal || opcode_jalr) begin
                if (opcode_jalr) a = i_rs1_data;
                rd_d = pc_d; // Store next PC in rd
                pc_d = sum; // Set PC to target address
            end 

            // Handle LUI instruction
            if (opcode_lui) rd_d = i_imm;

            // Handle AUIPC instruction
            if (opcode_auipc) rd_d = sum;

            // Handle CSR Instruction
            if(opcode_system && i_funct3!=0) rd_d = i_load_data_from_csr;

            // Disable write for certain instruction types
            if (opcode_branch || opcode_store || opcode_system && i_funct3 == 0) wr_rd_d = 0; // Added i_funct3==0 for non-csr system instructions
            else wr_rd_d = 1; // Enable write for other instruction types except when instruction is BRANCH/STORE/SYSTEM(Except CSR Instruction)
        end    
    end

/* Register outputs of this module for shorter combinational timing paths */
    always @(posedge i_clk,negedge i_rst_n) begin
        if(!i_rst_n) begin
            o_rd <= 0; 
            o_pc <= PC_RESET;
            o_wr_rd_en <= 0;
        end
        else begin
            o_rd <= rd_d;
            o_pc <= i_writeback_en ? pc_d:o_pc;
            o_wr_rd_en <= wr_rd_d && i_writeback_en; // Enable wr_rd iff current stage is WRITEBACK

        end
    end
endmodule
