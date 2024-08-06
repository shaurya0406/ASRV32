`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

// Complete SoC package containing the main CPU asrv32_core and Main Memory (Combined Instruction & Data Memory)
module asrv32_soc #(parameter PC_RESET=32'h00_00_00_00, MEMORY_DEPTH=1024) ( 
    input wire clk, // System clock
    input wire rst_n // Active low reset
    );
    
    // Instruction Memory Interface
    wire[31:0] inst; // Instruction from memory
    wire[31:0] iaddr; // Instruction address from CPU
    
    // Data Memory Interface
    wire[31:0] din; // Data retrieved from memory
    wire[31:0] dout; // Data to be stored to memory
    wire[31:0] daddr; // Address of data memory for store/load
    wire[3:0] wr_mask; // Write mask control
    wire wr_en; // Write enable 

    // Main ASRV32 core instantiation
    asrv32_core #(.PC_RESET(32'h00_00_00_00)) m0( 
        .i_clk(clk), // System clock
        .i_rst_n(rst_n), // Active low reset
        // Instruction Memory Interface
        .i_inst(inst), // 32-bit instruction
        .i_inst_addr(iaddr), // Address of instruction 
        // Data Memory Interface
        .i_data_from_memory(din), // Data retrieved from memory
        .o_store_data(dout), // Data to be stored to memory
        .o_store_data_addr(daddr), // Address of data memory for store/load
        .o_wr_mask(wr_mask), // Write mask control
        .o_wr_en(wr_en) // Write enable 
    );
        
    // Main memory instantiation
    main_memory #(.MEMORY_DEPTH(MEMORY_DEPTH)) m1(
        .clk(clk), // System clock
        // Instruction Memory Interface
        .inst_addr(iaddr[$clog2(MEMORY_DEPTH)-1:0]), // Instruction address
        .inst_out(inst), // Instruction output
        // Data Memory Interface
        .data_addr(daddr[$clog2(MEMORY_DEPTH)-1:0]), // Data address
        .data_in(dout), // Data input
        .wr_mask(wr_mask), // Write mask
        .wr_en(wr_en), // Write enable
        .data_out(din) // Data output
    );

endmodule


// Main memory module with combined instruction and data memory
module main_memory #(parameter MEMORY_DEPTH=1024) (
    input wire clk, // System clock
    // Instruction Memory Interface
    input wire[$clog2(MEMORY_DEPTH)-1:0] inst_addr, // Instruction address
    output wire[31:0] inst_out, // Instruction output
    // Data Memory Interface
    input wire[$clog2(MEMORY_DEPTH)-1:0] data_addr, // Data address
    input wire[31:0] data_in, // Data input
    input wire[3:0] wr_mask, // Write mask
    input wire wr_en, // Write enable
    output wire[31:0] data_out // Data output
);
    // Memory array for storing instructions and data
    reg[31:0] memory_regfile[MEMORY_DEPTH/4 - 1:0];
    integer i;
    
    // Initialize memory to zero
    initial begin 
        for(i = 0; i < MEMORY_DEPTH/4 - 1; i = i + 1) 
            memory_regfile[i] = 0; 
    end
    
    // Read instruction from memory
    assign inst_out = memory_regfile[{inst_addr >> 2}]; 
    // Read data from memory
    assign data_out = memory_regfile[data_addr[$clog2(MEMORY_DEPTH)-1:2]]; 
    
    // Write data to memory
    always @(posedge clk) begin
        if(wr_en) begin
            if(wr_mask[0]) memory_regfile[data_addr[$clog2(MEMORY_DEPTH)-1:2]][7:0] <= data_in[7:0]; 
            if(wr_mask[1]) memory_regfile[data_addr[$clog2(MEMORY_DEPTH)-1:2]][15:8] <= data_in[15:8];
            if(wr_mask[2]) memory_regfile[data_addr[$clog2(MEMORY_DEPTH)-1:2]][23:16] <= data_in[23:16];
            if(wr_mask[3]) memory_regfile[data_addr[$clog2(MEMORY_DEPTH)-1:2]][31:24] <= data_in[31:24];
        end        
    end
    
endmodule