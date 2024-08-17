`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* Fetch Module & Ports Declaration*/
module asrv32_fetch(
    /* Inputs */
    input wire i_clk,i_rst_n,       // Input Clock & Active Low Reset
    input wire i_fetch_stage_en,    // FETCH Enable
    input wire[31:0] i_inst,        // Input Instruction            
    /* Outputs */
    output reg[31:0] o_inst         // Output Instruction  
);

/* Initial Values for Stage and Instruction */
    initial begin
        o_inst = 0;     // Initial instruction is 0
    end

    reg[31:0] inst_d;   // Next instruction

/* Register Operation */
always @(posedge i_clk, negedge i_rst_n) begin
        if(!i_rst_n) begin
            o_inst <= 0;    // Reset instruction to 0
        end
        else begin
            o_inst <= i_inst;
            // o_inst <= i_fetch_stage_en ? i_inst : o_inst;   // Update current instruction if FETCH Enabled
        end
    end
endmodule