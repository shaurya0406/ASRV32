/*
This module provides a simple yet effective implementation of a register file controller for a RISC-V processor. 
Key points include:
- Ensuring that register 0 is always zero.
- Synchronous read and write operations controlled by clock enable signals.
- Efficient handling of read and write operations, with appropriate checks for register 0.
*/

`timescale 1ns/1ps
`default_nettype none

module asrv32_basereg
    (
        /* Inputs */
        input wire i_clk,           // Input Clock
        input wire i_ce_rd,         // Clock Enable (ce) for Synchronous Read
        input wire i_ce_wr,         // Clock Enable (ce) for Synchronous Write
        input wire[4:0] i_rs1_addr, // Address of the first source register. Total 32 registers from x0 to x31, hence addressable through 5 bits (2^5 = 32)
        input wire[4:0] i_rs2_addr, // Address of the second source register.
        input wire[4:0] i_rd_addr,  // Address of the destination register.
        input wire[31:0] i_rd_data, // Data to be written to the destination register.
        
        /* Outputs */
        input wire[31:0] o_rs1_data,    // Data read from the first source register.
        input wire[31:0] o_rs2_data     // Data read from the second source register.
    );

    /* Internal Signals, Registers and Base Register File Declaration */ 
    reg[4:0] rs1_addr_q =0, rs2_addr_q = 0;    // Registers to hold the addresses of the source registers.
    reg[31:0] base_regfile [31:1];     // 2D Array representing the base register file, where base_regfile[0] is hardwired to zero (it is not declared explicitly in the array because it is not used).
    wire reg_wr_en;
    
    /* Initialise Base Regfile to 0 for Testbench (not synthesizable) */
    initial begin
        for(i=0 ; i<32 ; i=i+1) base_regfile[i]=0; 
    end

    /* Sequential Logic for Read and Write Operations */ 
    always @(posedge i_clk) begin // On the positive edge of the clock
        if(reg_wr_en) begin // if 'reg_wr_en' signal is asserted, write the data 'i_rd_data' to the register addressed by 'i_rd_addr' (synchronous write).
            base_regfile[i_rd_addr] <= i_rd_data;
        end
        if(i_ce_rd) begin // If i_ce_read is high, capture the addresses of the source registers into rs1_addr_q and rs2_addr_q (synchronous read).
            rs1_addr_q <= i_rs1_addr; 
            rs2_addr_q <= i_rs2_addr;
        end
    end

    /* Combinational Logic for Control Signals using concurrent assignment*/
    assign reg_wr_en = (i_ce_wr && i_rd_addr != 0); // This signal is asserted if i_ce_wr is asserted and the destination address i_rd_addr is not zero (since register x0 is hardwired to zero).
    assign o_rs1_data = (rs1_addr_q == 0) ? 0 : base_regfile[rs1_addr_q];
    assign o_rs2_data = (rs2_addr_q == 0) ? 0 : base_regfile[rs2_addr_q];
endmodule

    


