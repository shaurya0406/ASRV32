`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* Fetch Module & Ports Declaration*/
module asrv32_fetch #(parameter PC_RESET = 32'h00_00_00_00) (
    /* Fetch Inputs */
    input wire i_clk,i_rst_n,       // Input Clock & Active Low Reset

    /* Memory (Wishbone Bus) */
    output wire o_stb_inst,         // Wishbone Bus Request/Strobe Signal to retrieve Instruction from memory
    output reg[31:0] o_inst_addr,   // Address of the Instruction to be requested from the memory
    input wire[31:0] i_inst,        // Input Instruction received from memory
    input wire i_ack_inst,          // Wishbone Bus Acknowledge Signal (Asserted if new instruction is now on the bus)

    output wire[31:0] o_inst_ifid,  // Current Instruction sent to the next stage of the pipeline (IF/ID Pipeline Register)
    output reg[31:0] o_pc_ifid,     // Current Instruction PC Value (IF/ID Pipeline Register)

    /* PC Control Management */
    input wire i_writeback_change_pc,       // Asserted when PC needs to change when going to trap or returning from trap
    input wire[31:0] i_writeback_next_pc,   // Next PC Value due to trap
    input wire i_alu_change_pc,             // Asserted when PC needs to change for taken branches and jumps
    input wire[31:0] i_alu_next_pc,         // Next PC due to branch or jump

    /* Pipeline control */
    output reg o_ce,    // Global clk enable for pipeline stalling of next stage
    input wire i_stall, // Global Stall logic for whole pipeline
    input wire i_flush  // Flush this stage
);

/* Intermediate Register Declaration: */

    // * inst_addr_d: Holds the next instruction address (PC) that will be used in the next clock cycle.
    // ? This is an intermediate value before it is assigned to the actual output `o_inst_addr`.
    reg[31:0] inst_addr_d;

    // * prev_pc: Stores the value of the previous PC before it gets updated.
    // ? This is used for alignment of the PC with the pipeline stages, ensuring the correct instruction fetch.
    reg[31:0] prev_pc;

    // * stall_fetch: Internal signal indicating whether the fetch stage should be stalled.
    // ? This is used to control the `ce` signal and prevent the fetch stage from updating the PC or fetching a new instruction when a stall is needed.
    reg stall_fetch;

    // * stall_q: A registered version of the stall signal used to detect and handle stalls in the pipeline.
    // ? It is used to store the current stall condition and helps in resuming the correct PC and instruction after the stall.
    reg stall_q;
    
    // * stalled_inst: Stores the instruction that was fetched but not yet processed when a stall occurs.
    // ? This allows the pipeline to resume execution of the same instruction after the stall is cleared.
    reg[31:0] stalled_inst;

    // * stalled_pc: Stores the PC value corresponding to the `stalled_inst` when a stall occurs.
    // ? This ensures the pipeline can resume execution from the correct instruction address after the stall.
    reg[31:0] stalled_pc;

    // * ce: Internal Clock Enable signal for the fetch stage. 
    // ? It controls whether the fetch stage is active in the current clock cycle.
    // ? If `ce` is high, the fetch stage is enabled; if low, the fetch stage is stalled.
    reg ce;

    // * ce_d: Delayed version of the `ce` signal for Pipeline control
    // ? It holds the value of `ce` that will be applied in the next clock cycle, ensuring proper timing for the PC update.
    reg ce_d;

/* Clk Enable Logic for this stage (IF) */
    always @(posedge i_clk, negedge i_rst_n) begin
        // Reset Condition
        if(!i_rst_n) ce <= 0; 
        // Insert Pipeline Bubble when PC needs to change by disabling clk enable for this stage so that instructions already in the pipeline (in-flight) dont execute
        else if((i_alu_change_pc || i_writeback_change_pc) && !(i_stall || stall_fetch)) ce <= 0; 
        // Enable Clock for this stage
        else ce <= 1;
    end

/* Request Instruction */
    assign o_stb_inst = ce; // Request for new instruction if this stage is enabled                                                               
    
/* Stall Logic for this stage (IF) 
    * - when next stages are stalled OR
    * - you have requested instruction but no acknowldege yet form the bus OR
    * - you have'nt requested at all (no request then no instruction to execute for this stage)
*/
    wire stall_bit = stall_fetch || i_stall || (o_stb_inst && !i_ack_inst) || !o_stb_inst; 

/* Combinational Logic for Program Counter (PC) and pipeline clock enable (ce) control */
    always @* begin
        inst_addr_d = 0;    // Default value for next instruction address (PC)
        ce_d = 0;           // Default value for clock enable signal
        
        // Stall the fetch stage when the next stages require it
        stall_fetch = i_stall;
        
        // * Prepare the next PC and create a pipeline bubble when the PC needs to change:
        // * This could happen due to exceptions, interrupts, or branch/jump instructions.
        
        // Handle PC change from writeback stage (e.g., due to traps or exceptions)
        if (i_writeback_change_pc) begin
            inst_addr_d = i_writeback_next_pc;  // Set PC to the new address provided by writeback stage
            ce_d = 0;                           // Disable clock enable signal for the next pipeline stage (create a bubble)
        end
        // Handle PC change from ALU stage (e.g., branch or jump instructions)
        else if (i_alu_change_pc) begin
            inst_addr_d = i_alu_next_pc;    // Set PC to the new address provided by ALU stage
            ce_d = 0;                       // Disable clock enable signal for the next pipeline stage (create a bubble)
        end
        // Normal sequential instruction fetch (increment PC by 4 bytes)
        else begin
            inst_addr_d = o_inst_addr + 32'd4;  // Increment PC to point to the next instruction
            ce_d = ce;                          // Maintain the current clock enable signal
        end
    end

/* Register the Outputs */
    always @(posedge i_clk, negedge i_rst_n) begin
        if(!i_rst_n) begin
            // On reset (active low), initialize key registers to 0/reset value
            o_ce <= 0;                  // Reset clock enable for the next stage
            o_inst_addr <= PC_RESET;    // Set instruction address to the reset value
            prev_pc <= PC_RESET;        // Initialize previous PC to reset value
            stalled_inst <= 0;          // Clear any stalled instruction data
            o_pc_ifid <= 0;             // Reset the current PC output to 0
        end
        else begin 
            // Update key registers if the fetch stage is enabled and next stages are not stalled
            if((ce && !stall_bit) || (stall_bit && !o_ce && ce) || i_writeback_change_pc) begin
                o_inst_addr <= inst_addr_d;                     // Update the instruction address with the next value
                o_pc_ifid <= stall_q ? stalled_pc : prev_pc;    // Update PC with stalled PC if stalled, else use prev_pc
                o_inst_ifid <= stall_q ? stalled_inst : i_inst; // Update instruction output with stalled instruction or current instruction
            end
            
            // If flush signal is asserted and no stall, disable clock enable for next stage
            if(i_flush && !stall_bit) begin
                o_ce <= 0;      // Flush the stage, preventing further execution in the next stage
            end
            // Otherwise, if not stalled, update the clock enable with delayed ce_d value
            else if(!stall_bit) begin
                o_ce <= ce_d;   // Propagate the clock enable signal to the next stage
            end
            // If there is a stall in this stage but the next stage is not stalled, create a pipeline bubble
            else if(stall_bit && !i_stall) begin
                o_ce <= 0;      // Disable clock enable for the next stage, creating a bubble
            end

            // Update the stall flag register to indicate whether the pipeline is stalled
            stall_q <= i_stall || stall_fetch;  // Set stall_q high if any stage is stalled
            
            // If a stall just occurred, store the current PC and instruction
            if(stall_bit && !stall_q) begin
                stalled_pc <= prev_pc;         // Store the current PC in stalled_pc
                stalled_inst <= i_inst;        // Store the current instruction in stalled_inst
            end
            
            // Update prev_pc with the current instruction address (o_inst_addr) for the next cycle
            prev_pc <= o_inst_addr;  // Align the PC value with the pipeline stages
        end
    end
endmodule