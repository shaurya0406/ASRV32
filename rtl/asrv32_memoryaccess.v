`timescale 1ns / 1ps
`default_nettype none

`include "asrv32_header.vh"

/* MemAccess Module & Ports Declaration */
module asrv32_memoryaccess(
    /* Inputs */
    input wire i_clk,   // Clock signal
    input wire i_rst_n, // Active-low reset signal
    
    /* EX Stage */
    input wire[31:0] i_rs2_data_exmem,            // Data to be stored to memory (always rs2)
    input wire[31:0] i_result_from_alu_exmem,     // Address of data to be stored or loaded (always from ALU)
    input wire[2:0] i_funct3_exmem,               // Determines data width (byte, half-word, word)
    input wire[`OPCODE_WIDTH-1:0] i_opcode_exmem, // Determines if o_store_data will be stored to data memory
    input wire[31:0] i_pc_exmem,                  // Pipeline Register
    
    /* Basereg Control */
    input wire i_wr_rd, // write rd to base reg is enabled (from memoryaccess stage)
    output reg o_wr_rd, // write rd to the base reg if enabled
    input wire[4:0] i_rd_addr, // address for destination register (from previous stage)
    output reg[4:0] o_rd_addr, // address for destination register
    input wire[31:0] i_rd, // value to be written back to destination reg
    output reg[31:0] o_rd, // value to be written back to destination register
    
    /* Data Memory Interface */
    output reg o_stb_data,                  // Wishbone request for read/write access to data memory
    input wire i_ack_data,                  // Wishbone ack by data memory (high when read data is ready or when write data is already written)
    input wire[31:0] i_data_from_memory,    // Data retrieved from memory
    output reg[31:0] o_store_data,          // Data to be stored to memory (mask-aligned)
    output reg[31:0] o_load_data,           // Data to be loaded to base register (zero or sign-extended)
    output reg[3:0] o_wr_mask,              // Write mask {byte3, byte2, byte1, byte0}
    output reg o_wr_mem_en,                 // Write to data memory if enabled

    /* WB Stage */
    output reg[31:0] o_result_from_alu_memwb,       // ALU Result used as Data Memory Address (Pipeline Reg)
    output reg[2:0] o_funct3_memwb,                 // funct3 passed to WB stage (byte,halfword,word) | (Pipeline Reg)
    output reg[`OPCODE_WIDTH-1:0] o_opcode_memwb,   // Opcode Type (Pipeline Reg)
    output reg[31:0] o_pc_memwb,                    // PC Value (Pipeline Reg)
    
    /* Pipeline Control */
    input wire i_stall_from_alu, // stalls this stage when incoming instruction is a load/store
    input wire i_ce,             // input clk enable for pipeline stalling of this stage
    output reg o_ce,             // output clk enable for pipeline stalling of next stage
    input wire i_stall,          // informs this stage to stall
    output reg o_stall,          // informs pipeline to stall
    input wire i_flush,          // flush this stage
    output reg o_flush           // flush previous stages
);

/* Intermediate Register Declarations: */
    reg[31:0] store_data_d; // Intermediate storage for o_store_data
    reg[31:0] load_data_d;  // Intermediate storage for o_load_data
    reg[3:0] wr_mask_d;     // Intermediate storage for wr_mask

    // Extract the last 2 bits of the address from the ALU result for byte/half-word addressing
    wire[1:0] addr_2 = i_result_from_alu_exmem[1:0]; // Last 2 bits of data memory address

/* Stall Logic for this stage (MEM) */
    wire stall_bit = i_stall || o_stall;

/* Data Load/Store Logic: */

    // Determine data to be loaded to base register or stored to data memory 
    always @* begin
        /*
        ? stall while data memory has not yet acknowledged i.e.wWrite data is not yet written or
        ? read data is not yet available. Don't stall when need to flush by next stage
        */
        o_stall = ((i_stall_from_alu && i_ce && !i_ack_data) || i_stall) && !i_flush;         
        o_flush = i_flush; //flush this stage along with previous stages
        
        store_data_d = 0;
        load_data_d = 0;
        wr_mask_d = 0; 
           
        case (i_funct3_exmem[1:0]) 
            2'b00: begin // Byte load/store

                    case(addr_2)  //choose which of the 4 byte will be loaded to basereg
                        2'b00: load_data_d = i_data_from_memory[7:0];
                        2'b01: load_data_d = i_data_from_memory[15:8];
                        2'b10: load_data_d = i_data_from_memory[23:16];
                        2'b11: load_data_d = i_data_from_memory[31:24];
                    endcase
                    load_data_d = {{{24{!i_funct3_exmem[2]}} & {24{load_data_d[7]}}} , load_data_d[7:0]}; //signed and unsigned extension
                    wr_mask_d = 4'b0001<<addr_2; //mask 1 of the 4 bytes
                    store_data_d = i_rs2_data_exmem<<{addr_2,3'b000}; //rs2<<(addr_2*8) , align data to mask
                   end
            2'b01: begin // Half-word load/store
                    load_data_d = addr_2[1]? i_data_from_memory[31:16]: i_data_from_memory[15:0]; //choose which of the 2 halfwords will be loaded to basereg
                    load_data_d = {{{16{!i_funct3_exmem[2]}} & {16{load_data_d[15]}}},load_data_d[15:0]}; //signed and unsigned extension
                    wr_mask_d = 4'b0011 << {addr_2[1], 1'b0}; // Mask upper or lower half-word
                    store_data_d = i_rs2_data_exmem << {addr_2[1], 4'b0000}; // Align data to mask
                   end
            2'b10: begin // Word load/store
                    load_data_d = i_data_from_memory;
                    wr_mask_d = 4'b1111; // Mask all bytes
                    store_data_d = i_rs2_data_exmem;
                   end
        endcase
    end

/* Register the outputs of this module */
    always @(posedge i_clk or negedge i_rst_n) begin
        if (!i_rst_n) begin
            o_store_data <= 0;
            o_load_data <= 0;
            o_wr_mask <= 0;
            o_wr_mem_en <= 0;
        end
        else begin
            o_store_data <= store_data_d;
            o_load_data <= load_data_d;
            o_wr_mask <= wr_mask_d;
            o_wr_mem_en <= i_opcode_exmem[`STORE] && i_memoryaccess_en; 
        end
    end 

endmodule