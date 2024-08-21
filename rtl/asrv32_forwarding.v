`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* Forwarding Module 
 * This module implements operand forwarding to resolve data hazards 
 * in a pipelined RISC-V processor. Operand forwarding allows the 
 * use of the most recent data value from the pipeline stages 
 * instead of waiting for the data to be written back to the 
 * register file, thereby preventing stalls and improving performance.
 */

/* Decoder Module & Ports Declaration */
module asrv32_forwarding (
    input wire[31:0] i_rs1_orig, // Original rs1 value fetched from base register file
    input wire[31:0] i_rs2_orig, // Original rs2 value fetched from base register file
    input wire[4:0] i_decoder_rs1_addr_q, // Address of operand rs1 being used in EX stage
    input wire[4:0] i_decoder_rs2_addr_q, // Address of operand rs2 being used in EX stage
    output reg o_alu_force_stall, // High to force ALU stage to stall when necessary
    output reg[31:0] o_rs1, // Forwarded rs1 value after applying operand forwarding
    output reg[31:0] o_rs2, // Forwarded rs2 value after applying operand forwarding
    // Stage 4 [MEMORYACCESS]
    input wire[4:0] i_alu_rd_addr, // Destination register address from EX stage (stage 4)
    input wire i_alu_wr_rd, // High if rd_addr will be written at stage 4
    input wire i_alu_rd_valid, // High if rd is already valid at stage 4 (not a LOAD nor CSR instruction)
    input wire[31:0] i_alu_rd, // rd value in EX stage (stage 4)
    input wire i_memoryaccess_ce, // High if stage 4 (Memory Access) is enabled
    // Stage 5 [WRITEBACK]
    input wire[4:0] i_memoryaccess_rd_addr, // Destination register address from Memory Access stage (stage 5)
    input wire i_memoryaccess_wr_rd, // High if rd_addr will be written at stage 5
    input wire[31:0] i_writeback_rd, // rd value in Writeback stage (stage 5)
    input wire i_writeback_ce // High if stage 5 (Writeback) is enabled
);

/* Combinational Logic
 * Always block to determine the appropriate rs1 and rs2 values
 * considering potential data hazards, and to manage ALU stalls 
 * if necessary. 
*/
    always @* begin
        o_rs1 = i_rs1_orig; // Start with the original rs1 value from base register file
        o_rs2 = i_rs2_orig; // Start with the original rs2 value from base register file
        o_alu_force_stall = 0; // Initialize ALU stall signal to zero (no stall)

/* Data Hazard Detection for rs1:
 * If rs1 is the destination register of an instruction currently in the pipeline (ALU or Memory Access stage),
 * the module checks if forwarding is necessary.
 */

        // Operand Forwarding for rs1 when the next value of rs1 is currently in the ALU stage (stage 4)
        if ((i_decoder_rs1_addr_q == i_alu_rd_addr) && i_alu_wr_rd && i_memoryaccess_ce) begin 
            if (!i_alu_rd_valid) begin   // If the next value of rs1 comes from a LOAD or CSR instruction, stall ALU until valid
                o_alu_force_stall = 1;   // Force ALU stage to stall until stage 4 is disabled and rs1 becomes available at stage 5
            end
            o_rs1 = i_alu_rd; // Forward the ALU stage (stage 4) result to rs1
        end
        // Operand Forwarding for rs1 when the next value of rs1 is currently in the Writeback stage (stage 5)
        else if ((i_decoder_rs1_addr_q == i_memoryaccess_rd_addr) && i_memoryaccess_wr_rd && i_writeback_ce) begin 
            o_rs1 = i_writeback_rd; // Forward the Writeback stage (stage 5) result to rs1
        end
        
/* Data Hazard Detection for rs2:
 * Similar to rs1, checks if rs2 needs to be forwarded from an earlier pipeline stage.
 */

        // Operand Forwarding for rs2 when the next value of rs2 is currently in the ALU stage (stage 4)
        if ((i_decoder_rs2_addr_q == i_alu_rd_addr) && i_alu_wr_rd && i_memoryaccess_ce) begin 
            if (!i_alu_rd_valid) begin   // If the next value of rs2 comes from a LOAD or CSR instruction, stall ALU until valid
                o_alu_force_stall = 1;   // Force ALU stage to stall until stage 4 is disabled and rs2 becomes available at stage 5
            end
            o_rs2 = i_alu_rd; // Forward the ALU stage (stage 4) result to rs2
        end
        // Operand Forwarding for rs2 when the next value of rs2 is currently in the Writeback stage (stage 5)
        else if ((i_decoder_rs2_addr_q == i_memoryaccess_rd_addr) && i_memoryaccess_wr_rd && i_writeback_ce) begin 
            o_rs2 = i_writeback_rd; // Forward the Writeback stage (stage 5) result to rs2
        end

/* Logic for x0 (Special Case)
 * Handle the special case where the register address is zero (x0).
 * The x0 register is hardwired to zero, so no forwarding is necessary.
 */
        if (i_decoder_rs1_addr_q == 0) o_rs1 = 0;
        if (i_decoder_rs2_addr_q == 0) o_rs2 = 0;
    end

endmodule
