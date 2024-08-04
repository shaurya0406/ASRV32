`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* Top Core Module & Ports Declaration*/

module asrv32_core #(parameter PC_RESET = 32'h00_00_00_00) ( 
    /* Inputs */
    input wire i_clk, i_rst_n,
    
    /* Instruction Memory Interface (32 bit rom) */
    input wire[31:0] i_inst, //32-bit instruction

    output wire[31:0] i_inst_addr, //address of instruction 

    /* Data Memory Interface (32 bit ram) */
    input wire[31:0] i_data_from_memory, //data retrieve from memory

    output wire[31:0] o_store_data, //data to be stored to memory
    output wire[31:0] o_store_data_addr, //address of data memory for store/load
    output wire[3:0] o_wr_mask, //write mask control
    output wire o_wr_en //write enable 
);

    //wires for basereg
    wire[31:0] rs1_data, rs2_data, rd_data; //value of source register 1 and 2 and destination register 

    //wires for asrv32_decoder
    wire[31:0] imm; 
    wire[4:0] rs1_addr, rs2_addr;
    wire[4:0] rd_addr; 
    wire[2:0] funct3;
    wire [`ALU_WIDTH-1:0] alu_op;
    wire [`OPCODE_WIDTH-1:0] opcode;

    //wires for asrv32_alu
    wire[31:0] op1,op2;
    wire[31:0] alu_result;

    //wires for asrv32_writeback
    wire[31:0] load_data; //data to be loaded to base reg
    wire[31:0] pc; //program counter (PC) value
    wire wr_rd_en; //write to rd if enabled

    //wires for asrv32_fsm
    wire[31:0] inst_q;
    wire[2:0] stage_q;
    wire alu_stage_en;
    wire memoryaccess_stage_en;
    wire writeback_stage_en; 
    
    //address of memories 
    assign i_inst_addr = pc; //instruction address
    assign o_store_data_addr = alu_result; //data address
  
    //module instantiations (all outputs are registered)
    asrv32_basereg m0( //regfile controller for the 32 integer base registers
        .i_clk(i_clk),  // Input Clock
        .i_ce_rd(1'b1), // Enable Clock for Synchronous Read by Default
        .i_ce_wr(wr_rd_en), // Clock Enable for Synchronous Write
        .i_rs1_addr(rs1_addr), //source register 1 address
        .i_rs2_addr(rs2_addr), //source register 2 address
        .i_rd_addr(rd_addr), //destination register address
        .i_rd_data(rd_data), //data to be written to destination register
        .o_rs1_data(rs1_data), //source register 1 value (Compiler Warning: input port o_rs1_data is coerced to inout.)
        .o_rs2_data(rs2_data) //source register 2 value (Compiler Warning: input port o_rs2_data is coerced to inout.)
    );
    
    asrv32_decoder m1( //logic for the decoding of the 32 bit instruction [DECODE STAGE]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_inst(inst_q), //32 bit instruction
        .o_rs1_addr(rs1_addr),//address for register source 1
        .o_rs2_addr(rs2_addr), //address for register source 2
        .o_rd_addr(rd_addr), //address for destination address
        .o_imm(imm), //extended value for immediate
        .o_funct3(funct3), //function type
        .o_opcode(opcode),
        .o_alu_op(alu_op)
    );

    asrv32_alu m2( //ALU combinational logic [EXECUTE STAGE]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_alu_en(alu_stage_en), //update alu output iff stage is currently on EXECUTE (ALU stage)
        .i_alu(alu_op),
        .i_op1(op1), //rs1 or pc
        .i_op2(op2), //rs2 or imm 
        .o_alu_result(alu_result) //result of arithmetic operation  
    );
    
    asrv32_memoryaccess m3( //logic controller for data memory access (load/store) [MEMORY STAGE]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_memoryaccess_en(memoryaccess_stage_en), //enable wr_mem iff stage is currently on LOADSTORE
        .i_rs2_data(rs2_data), //data to be stored to memory is always rs2
        .i_data_from_memory(i_data_from_memory), //data retrieve from memory 
        .i_result_from_alu(alu_result), //ALU Result
        .i_funct3(funct3), //byte,half-word,word
        .i_opcode(opcode), //determines if data_store will be to stored to data memory
        .o_store_data(o_store_data), //data to be stored to memory (mask-aligned)
        .o_load_data(load_data), //data to be loaded to base reg (z-or-s extended) 
        .o_wr_mask(o_wr_mask), //write mask {byte3,byte2,byte1,byte0}
        .o_wr_mem_en(o_wr_en) //write to data memory if enabled
    );
    
    asrv32_writeback #(.PC_RESET(PC_RESET)) m4( //logic controller for the next PC and rd value [WRITEBACK STAGE]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_writeback_en(writeback_stage_en), //enable wr_rd iff stage is currently on WRITEBACK
        .i_result_from_alu(alu_result), //output of ALU
        .i_imm(imm), //immediate value
        .i_rs1_data(rs1_data), //source register 1 value
        .i_load_data_from_memory(load_data), //data to be loaded to base reg
        .i_opcode(opcode),
        .o_rd(rd_data), //value to be written back to destination register
        .o_pc(pc), //new PC value
        .o_wr_rd_en(wr_rd_en) //write rd to the base reg if enabled
    );

    asrv32_fsm m5( //FSM controller for the fetch, decode, execute, memory access, and writeback processes.
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_inst(i_inst), //instruction
        .i_pc(pc), //Program Counter
        .i_rs1_data(rs1_data), ////Source register 1 value
        .i_rs2_data(rs2_data), //Source Register 2 value
        .i_imm(imm), //Immediate value
        .i_opcode(opcode),
        .o_inst_q(inst_q), //registered instruction
        .o_stage_q(stage_q), //current stage
        .o_op1(op1), //value of op1 in ALU
        .o_op2(op2), //value of op2 in ALU
        .o_alu_stage_en(alu_stage_en),//high if stage is on EXECUTE
        .o_memoryaccess_stage_en(memoryaccess_stage_en),//high if stage is on MEMORYACCESS
        .o_writeback_stage_en(writeback_stage_en) //high if stage is on WRITEBACK
    );
      
endmodule

