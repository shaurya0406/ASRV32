`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* FSM Module & Ports Declaration */
module asrv32_fsm (
    /* Inputs */
    input wire i_clk,                       // Clock input
    input wire i_rst_n,                     // Active-low reset input
    input wire[31:0] i_inst,                // Instruction input
    input wire[31:0] i_pc,                  // Program Counter input
    input wire[31:0] i_rs1_data,            // Source register 1 value input
    input wire[31:0] i_rs2_data,            // Source register 2 value input
    input wire[31:0] i_imm,                 // Immediate value input
    input wire[`OPCODE_WIDTH-1:0] i_opcode, // Opcode input

    /* Outputs */
    output reg[31:0] o_inst_q,              // Registered instruction output
    output reg[2:0] o_stage_q,              // Current stage output
    output reg[31:0] o_op1,                 // ALU operand 1 output
    output reg[31:0] o_op2,                 // ALU operand 2 output
    output wire o_alu_stage_en,             // ALU stage enable signal
    output wire o_memoryaccess_stage_en,    // Memory access stage enable signal
    output wire o_writeback_stage_en,       // Writeback stage enable signal
    output wire o_csr_stage_en,             // CSR Stage enable signal (high if stage is on EXECUTE, CSR Operations take place between EXECUTE and MEMACCESS Stage as they require address calculated by the ALU)
    output wire o_done_tick                 // High for one clock cycle at the end of every instruction for MINSTRET CSR 
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

/* FSM State Definitions */
    localparam  FETCH = 0,        // Fetch stage
                DECODE = 1,       // Decode stage
                EXECUTE = 2,      // Execute stage
                MEMORYACCESS = 3, // Memory access stage
                WRITEBACK = 4;    // Writeback stage

/* Initial Values for Stage and Instruction */
    initial begin
        o_stage_q = FETCH; // Initial stage is FETCH
        o_inst_q = 0;      // Initial instruction is 0
    end
    
    reg[2:0] stage_d;   // Next stage
    reg[31:0] inst_d;   // Next instruction

/* FSM Logic for 5-stage processor (unpipelined) */  
    always @* begin
        stage_d = o_stage_q; // Default next stage is current stage
        inst_d = o_inst_q;   // Default next instruction is current instruction
        o_op1 = 0;           // Default ALU operand 1 is 0
        o_op2 = 0;           // Default ALU operand 2 is 0
        
        case(o_stage_q)
            FETCH:  begin // Fetch the instruction
                            inst_d = i_inst;        // Capture the instruction
                            stage_d = DECODE;       // Move to Decode stage
                    end

            DECODE:         stage_d = EXECUTE;      // Move to Execute stage after decoding

            EXECUTE:begin // Execute ALU operation
                            // o_op1 = (opcode_jal || opcode_auipc)? i_pc : i_rs1_data;      // Determine ALU operand 1
                            // o_op2 = (opcode_rtype || opcode_branch)? i_rs2_data : i_imm;  // Determine ALU operand 2
                            stage_d = MEMORYACCESS; // Move to Memory Access stage
                    end
                  
            MEMORYACCESS:   stage_d = WRITEBACK;    // Move to Writeback stage after memory access

            WRITEBACK:      stage_d = FETCH;        // Move back to Fetch stage

            default:        stage_d = FETCH;        // Default stage is Fetch
        endcase
    end

/* Register Operation */
    always @(posedge i_clk, negedge i_rst_n) begin
        if(!i_rst_n) begin
            o_stage_q <= FETCH;     // Reset stage to Fetch
            o_inst_q <= 0;          // Reset instruction to 0
        end
        else begin
            o_stage_q <= stage_d;   // Update current stage
            o_inst_q <= inst_d;     // Update current instruction
        end
    end

/* Stage Enable Signals */
    assign o_alu_stage_en = o_stage_q == EXECUTE;                       // ALU stage enable
    assign o_memoryaccess_stage_en = o_stage_q == MEMORYACCESS;         // Memory access stage enable
    assign o_writeback_stage_en = o_stage_q == WRITEBACK;               // Writeback stage enable
    assign o_csr_stage_en = o_stage_q == MEMORYACCESS;                  // CSR Stage enable (Asserted when next stage is MEMACCESS and current stage is EXECUTE)
    assign o_done_tick = stage_d == FETCH && o_stage_q == WRITEBACK;    // One instruction is executed
    
endmodule
