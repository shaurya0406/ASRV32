`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

// Complete SoC package containing the main CPU asrv32_core and Main Memory (Combined Instruction & Data Memory)
module asrv32_soc #(parameter CLK_FREQ_MHZ=100, PC_RESET=32'h00_00_00_00, TRAP_ADDRESS=32'h00_00_00_00, MEMORY_DEPTH=1024) ( 
    input wire clk, // System clock
    input wire rst_n, // Active low reset
    //Interrupts
    input wire i_external_interrupt, //interrupt from external source
    input wire i_software_interrupt, //interrupt from software
    // Timer Interrupt
    input wire i_mtime_wr, //write to mtime
    input wire i_mtimecmp_wr,  //write to mtimecmp
    input wire[63:0] i_mtime_din, //data to be written to mtime
    input wire[63:0] i_mtimecmp_din //data to be written to mtimecmp
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
    asrv32_core #(.PC_RESET(PC_RESET),.CLK_FREQ_MHZ(CLK_FREQ_MHZ), .TRAP_ADDRESS(TRAP_ADDRESS)) m0( 
        .i_clk(clk), // System clock
        .i_rst_n(rst_n), // Active low reset
        // Instruction Memory Interface
        .i_inst(inst), // 32-bit instruction
        .o_inst_addr(iaddr), // Address of instruction 
        // Data Memory Interface
        .i_data_from_memory(din), // Data retrieved from memory
        .o_store_data(dout), // Data to be stored to memory
        .o_store_data_addr(daddr), // Address of data memory for store/load
        .o_wr_mask(wr_mask), // Write mask control
        .o_wr_en(wr_en), // Write enable 
        //Interrupts
        .i_external_interrupt(i_external_interrupt), //interrupt from external source
        .i_software_interrupt(i_software_interrupt), //interrupt from software
        // Timer Interrupt
        .i_mtime_wr(i_mtime_wr), //write to mtime
        .i_mtimecmp_wr(i_mtimecmp_wr),  //write to mtimecmp
        .i_mtime_din(i_mtime_din), //data to be written to mtime
        .i_mtimecmp_din(i_mtimecmp_din) //data to be written to mtimecmp
    );
        
    // Main memory instantiation
    main_memory #(.MEMORY_DEPTH(MEMORY_DEPTH)) m1(
        .i_clk(clk), // System clock
        // Instruction Memory Interface
        .i_inst_addr(iaddr[$clog2(MEMORY_DEPTH)-1:0]), // Instruction address
        .o_inst_out(inst), // Instruction output
        // Data Memory Interface
        .i_data_addr(daddr[$clog2(MEMORY_DEPTH)-1:0]), // Data address
        .i_data_in(dout), // Data input
        .i_wr_mask(wr_mask), // Write mask
        .i_wr_en(wr_en), // Write enable
        .o_data_out(din) // Data output
    );

endmodule


// Main memory module with combined instruction and data memory
module main_memory #(parameter MEMORY_DEPTH=1024) (
    input wire i_clk, // System clock
    // Instruction Memory Interface
    input wire[$clog2(MEMORY_DEPTH)-1:0] i_inst_addr, // Instruction address
    output wire[31:0] o_inst_out, // Instruction output
    // Data Memory Interface
    input wire[$clog2(MEMORY_DEPTH)-1:0] i_data_addr, // Data address
    input wire[31:0] i_data_in, // Data input
    input wire[3:0] i_wr_mask, // Write mask
    input wire i_wr_en, // Write enable
    output wire[31:0] o_data_out // Data output
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
    assign o_inst_out = memory_regfile[{i_inst_addr >> 2}]; 
    // Read data from memory
    assign o_data_out = memory_regfile[i_data_addr[$clog2(MEMORY_DEPTH)-1:2]]; 
    
    // Write data to memory
    always @(posedge i_clk) begin
        if(i_wr_en) begin
            if(i_wr_mask[0]) memory_regfile[i_data_addr[$clog2(MEMORY_DEPTH)-1:2]][7:0] <= i_data_in[7:0]; 
            if(i_wr_mask[1]) memory_regfile[i_data_addr[$clog2(MEMORY_DEPTH)-1:2]][15:8] <= i_data_in[15:8];
            if(i_wr_mask[2]) memory_regfile[i_data_addr[$clog2(MEMORY_DEPTH)-1:2]][23:16] <= i_data_in[23:16];
            if(i_wr_mask[3]) memory_regfile[i_data_addr[$clog2(MEMORY_DEPTH)-1:2]][31:24] <= i_data_in[31:24];
        end        
    end
    
endmodule