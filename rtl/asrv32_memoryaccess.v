`timescale 1ns / 1ps
`default_nettype none

`include "asrv32_header.vh"

/* MemAccess Module & Ports Declaration */
module asrv32_memoryaccess(
    /* Inputs */
    input wire i_clk,                       // Clock signal
    input wire i_rst_n,                     // Active-low reset signal
    input wire i_memoryaccess_en,           // Enable write memory if stage is currently on MEMORYACCESS
    input wire[31:0] i_rs2_data,            // Data to be stored to memory (always rs2)
    input wire[31:0] i_data_from_memory,    // Data retrieved from memory
    input wire[31:0] i_result_from_alu,     // Address of data to be stored or loaded (always from ALU)
    input wire[2:0] i_funct3,               // Determines data width (byte, half-word, word)
    input wire[`OPCODE_WIDTH-1:0] i_opcode, // Determines if o_store_data will be stored to data memory

    /* Outputs */
    output reg[31:0] o_store_data,          // Data to be stored to memory (mask-aligned)
    output reg[31:0] o_load_data,           // Data to be loaded to base register (zero or sign-extended)
    output reg[3:0] o_wr_mask,              // Write mask {byte3, byte2, byte1, byte0}
    output reg o_wr_mem_en                  // Write to data memory if enabled
);

    /* Intermediate Register Declarations: */
    reg[31:0] store_data_d; // Intermediate storage for o_store_data
    reg[31:0] load_data_d;  // Intermediate storage for o_load_data
    reg[3:0] wr_mask_d;     // Intermediate storage for wr_mask

    // Extract the last 2 bits of the address from the ALU result for byte/half-word addressing
    wire[1:0] addr_2 = i_result_from_alu[1:0]; // Last 2 bits of data memory address

    /* Data Load/Store Logic: */

    // Determine data to be loaded to base register or stored to data memory 
    always @* begin
        store_data_d = 0;
        load_data_d = 0;
        wr_mask_d = 0; 
           
        case (i_funct3[1:0]) 
            2'b00: begin // Byte load/store

                    case(addr_2)  //choose which of the 4 byte will be loaded to basereg
                        2'b00: load_data_d = i_data_from_memory[7:0];
                        2'b01: load_data_d = i_data_from_memory[15:8];
                        2'b10: load_data_d = i_data_from_memory[23:16];
                        2'b11: load_data_d = i_data_from_memory[31:24];
                    endcase
                    load_data_d = {{{24{!i_funct3[2]}} & {24{load_data_d[7]}}} , load_data_d[7:0]}; //signed and unsigned extension
                    wr_mask_d = 4'b0001<<addr_2; //mask 1 of the 4 bytes
                    store_data_d = i_rs2_data<<{addr_2,3'b000}; //rs2<<(addr_2*8) , align data to mask
                   end
            2'b01: begin // Half-word load/store
                    load_data_d = addr_2[1]? i_data_from_memory[31:16]: i_data_from_memory[15:0]; //choose which of the 2 halfwords will be loaded to basereg
                    load_data_d = {{{16{!i_funct3[2]}} & {16{load_data_d[15]}}},load_data_d[15:0]}; //signed and unsigned extension
                    wr_mask_d = 4'b0011 << {addr_2[1], 1'b0}; // Mask upper or lower half-word
                    store_data_d = i_rs2_data << {addr_2[1], 4'b0000}; // Align data to mask
                   end
            2'b10: begin // Word load/store
                    load_data_d = i_data_from_memory;
                    wr_mask_d = 4'b1111; // Mask all bytes
                    store_data_d = i_rs2_data;
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
            o_wr_mem_en <= i_opcode[`STORE] && i_memoryaccess_en; 
        end
    end 

endmodule