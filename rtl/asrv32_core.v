`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* Top Core Module & Ports Declaration*/

module asrv32_core #(parameter PC_RESET = 32'h00_00_00_00, TRAP_ADDRESS = 0, ZICSR_EXTENSION = 1) ( 
    /* Inputs */
    input wire i_clk, i_rst_n,
    
    /* Instruction Memory Interface (32 bit rom) */
    input wire[31:0] i_inst, // 32-bit instruction
    
    output wire[31:0] o_inst_addr, //address of instruction 
    
    output wire o_stb_inst, // Request for read access to instruction memory
    input wire i_ack_inst, //ack (high if new instruction is ready)

    /* Data Memory Interface (32 bit ram) */
    
    input wire[31:0] i_data_from_memory, //data retrieve from memory

    output wire[31:0] o_store_data, //data to be stored to memory
    output wire[31:0] o_store_data_addr, //address of data memory for store/load
    output wire[3:0] o_wr_mask, //write mask control
    output wire o_wr_en, //write enable 

    output wire o_stb_data, //request for read/write access to data memory
    input wire i_ack_data, //ack by data memory (high when read data is ready or when write data is already written)

    /* Interrupts */
    input wire i_external_interrupt, //interrupt from external source
    input wire i_software_interrupt, //interrupt from software
    // TODO: Memory Mapped Timer Interrupt */
    input wire i_timer_interrupt //interrupt from timer
    // ! Moved to Top Soc
    // input wire i_mtime_wr, //write to mtime
    // input wire i_mtimecmp_wr,  //write to mtimecmp
    // input wire[63:0] i_mtime_din, //data to be written to mtime
    // input wire[63:0] i_mtimecmp_din //data to be written to mtimecmp
);

    // wires for Fetch
    wire[31:0] fetch_pc;
    wire[31:0] fetch_inst;

    //wires for basereg
    wire[31:0] rs1_orig,rs2_orig;   
    wire[31:0] rs1,rs2;  
    wire ce_read;

    //wires for asrv32_decoder
    wire[`ALU_WIDTH-1:0] decoder_alu;
    wire[`OPCODE_WIDTH-1:0] decoder_opcode;
    wire[31:0] decoder_pc;
    wire[4:0] decoder_rs1_addr, decoder_rs2_addr;
    wire[4:0] decoder_rs1_addr_q, decoder_rs2_addr_q;
    wire[4:0] decoder_rd_addr; 
    wire[31:0] decoder_imm; 
    wire[2:0] decoder_funct3;
    wire[`EXCEPTION_WIDTH-1:0] decoder_exception;
    wire decoder_ce;
    wire decoder_flush;

    //wires for asrv32_alu
    wire[`OPCODE_WIDTH-1:0] alu_opcode;
    wire[4:0] alu_rs1_addr;
    wire[31:0] alu_rs1;
    wire[31:0] alu_rs2;
    wire[11:0] alu_imm;
    wire[2:0] alu_funct3;
    wire[31:0] alu_y;
    wire[31:0] alu_pc;
    wire[31:0] alu_next_pc;
    wire alu_change_pc;
    wire alu_wr_rd;
    wire[4:0] alu_rd_addr;
    wire[31:0] alu_rd;
    wire alu_rd_valid;
    wire[`EXCEPTION_WIDTH-1:0] alu_exception;
    wire alu_ce;
    wire alu_flush;
    wire alu_force_stall;

    //wires for asrv32_memoryaccess
    wire[`OPCODE_WIDTH-1:0] memoryaccess_opcode;
    wire[2:0] memoryaccess_funct3;
    wire[31:0] memoryaccess_pc;
    wire memoryaccess_wr_rd;
    wire[4:0] memoryaccess_rd_addr;
    wire[31:0] memoryaccess_rd;
    wire[31:0] memoryaccess_data_load;
    wire memoryaccess_wr_mem;
    wire memoryaccess_ce;
    wire memoryaccess_flush;
    wire o_stall_from_alu;

    //wires for asrv32_writeback
    wire writeback_wr_rd; 
    wire[4:0] writeback_rd_addr; 
    wire[31:0] writeback_rd;
    wire[31:0] writeback_next_pc;
    wire writeback_change_pc;
    wire writeback_ce;
    wire writeback_flush;

    // !wires for asrv32_fsm
    // // wire[31:0] inst_q;
    // wire[2:0] stage_q;
    // wire fetch_stage_en;
    // wire alu_stage_en;
    // wire memoryaccess_stage_en;
    // wire writeback_stage_en; 
    // wire csr_stage_en;
    // wire done_tick;
    
    //wires for asrv32_csr
    wire[31:0] csr_out; //CSR value to be stored to basereg
    wire[31:0] csr_return_address; //mepc CSR
    wire[31:0] csr_trap_address; //mtvec CSR
    wire csr_go_to_trap; //high before going to trap (if exception/interrupt detected)
    wire csr_return_from_trap; //high before returning from trap (via mret)

    // Stall from each stage
    wire stall_decoder,
         stall_alu,
         stall_memoryaccess,
         stall_writeback; //control stall of each pipeline stages

    assign ce_read = decoder_ce && !stall_decoder; //reads basereg only decoder is not stalled 
    assign o_wr_en = memoryaccess_wr_mem && !writeback_change_pc; 

    //module instantiations (all outputs are registered)

    asrv32_forwarding operand_forwarding ( //logic for operand forwarding
        .i_rs1_orig(rs1_orig), //current rs1 value saved in basereg
        .i_rs2_orig(rs2_orig), //current rs2 value saved in basereg
        .i_decoder_rs1_addr_q(decoder_rs1_addr_q), //address of operand rs1 used in ALU stage
        .i_decoder_rs2_addr_q(decoder_rs2_addr_q), //address of operand rs2 used in ALU stage
        .o_alu_force_stall(alu_force_stall), //high to force ALU stage to stall
        .o_rs1(rs1), //rs1 value with Operand Forwarding
        .o_rs2(rs2), //rs2 value with Operand Forwarding
        // Stage 4 [MEMORYACCESS]
        .i_alu_rd_addr(alu_rd_addr), //destination register address
        .i_alu_wr_rd(alu_wr_rd), //high if rd_addr will be written
        .i_alu_rd_valid(alu_rd_valid), //high if rd is already valid at this stage (not LOAD nor CSR instruction)
        .i_alu_rd(alu_rd), //rd value in stage 4
        .i_memoryaccess_ce(memoryaccess_ce), //high if stage 4 is enabled
        // Stage 5 [WRITEBACK]
        .i_memoryaccess_rd_addr(memoryaccess_rd_addr), //destination register address
        .i_memoryaccess_wr_rd(memoryaccess_wr_rd), //high if rd_addr will be written
        .i_writeback_rd(writeback_rd), //rd value in stage 5
        .i_writeback_ce(writeback_ce) //high if stage 4 is enabled
    );

    asrv32_basereg m0( //regfile controller for the 32 integer base registers
        .i_clk(i_clk),
        .i_ce_rd(ce_read), //clock enable for reading from basereg [STAGE 2]
        .i_rs1_addr(decoder_rs1_addr), //source register 1 address
        .i_rs2_addr(decoder_rs2_addr), //source register 2 address
        .i_rd_addr(writeback_rd_addr), //destination register address
        .i_rd_data(writeback_rd), //data to be written to destination register
        .i_ce_wr(writeback_wr_rd), //write enable
        .o_rs1_data(rs1_orig), //source register 1 value
        .o_rs2_data(rs2_orig) //source register 2 value
    );
    
    asrv32_fetch #(.PC_RESET(PC_RESET)) m1( // logic for fetching instruction [FETCH STAGE , STAGE 1]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .o_inst_addr(o_inst_addr), //Instruction address
        .o_pc_ifid(fetch_pc), //PC value of o_inst
        .i_inst(i_inst), // retrieved instruction from Memory
        .o_inst_ifid(fetch_inst), // instruction
        .o_stb_inst(o_stb_inst), // request for instruction
        .i_ack_inst(i_ack_inst), //ack (high if new instruction is ready)
        // PC Control
        .i_writeback_change_pc(writeback_change_pc), //high when PC needs to change when going to trap or returning from trap
        .i_writeback_next_pc(writeback_next_pc), //next PC due to trap
        .i_alu_change_pc(alu_change_pc), //high when PC needs to change for taken branches and jumps
        .i_alu_next_pc(alu_next_pc), //next PC due to branch or jump
        /// Pipeline Control ///
        .o_ce(decoder_ce), // output clk enable for pipeline stalling of next stage
        .i_stall((stall_decoder || stall_alu || stall_memoryaccess || stall_writeback)), //informs this stage to stall
        .i_flush(decoder_flush) //flush this stage
    ); 
  
    asrv32_decoder m2( //logic for the decoding of the 32 bit instruction [DECODE STAGE , STAGE 2]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_inst_ifid(fetch_inst), //32 bit instruction
        .i_pc_ifid(fetch_pc), //PC value from fetch stage
        .o_pc_idex(decoder_pc), //PC value
        .o_rs1_addr(decoder_rs1_addr),// address for register source 1
        .o_rs1_addr_idex(decoder_rs1_addr_q), // registered address for register source 1
        .o_rs2_addr(decoder_rs2_addr), // address for register source 2
        .o_rs2_addr_idex(decoder_rs2_addr_q), // registered address for register source 2
        .o_rd_addr_idex(decoder_rd_addr), // address for destination register
        .o_imm_idex(decoder_imm), // extended value for immediate
        .o_funct3_idex(decoder_funct3), // function type
        .o_alu_op_idex(decoder_alu), //alu operation type
        .o_opcode_idex(decoder_opcode), //opcode type
        .o_exception_idex(decoder_exception), //exceptions: illegal inst, ecall, ebreak, mret
         /// Pipeline Control ///
        .i_ce(decoder_ce), // input clk enable for pipeline stalling of this stage
        .o_ce(alu_ce), // output clk enable for pipeline stalling of next stage
        .i_stall((stall_alu || stall_memoryaccess || stall_writeback)), //informs this stage to stall
        .o_stall(stall_decoder), //informs pipeline to stall
        .i_flush(alu_flush), //flush this stage
        .o_flush(decoder_flush) //flushes previous stages
    );

    asrv32_alu m3( //ALU combinational logic [EXECUTE STAGE , STAGE 3]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_alu_op_idex(decoder_alu), //alu operation type
        .i_rs1_addr_idex(decoder_rs1_addr_q), //address for register source 1
        .o_rs1_addr_excsr(alu_rs1_addr), //address for register source 1
        .i_rs1_data(rs1), //Source register 1 value
        .o_rs1_data_excsr(alu_rs1), //Source register 1 value
        .i_rs2_data(rs2), //Source Register 2 value
        .o_rs2_data_exmem(alu_rs2), //Source Register 2 value
        .i_imm_idex(decoder_imm), //Immediate value from previous stage
        .o_imm_excsr(alu_imm), //Immediate value
        .i_funct3_idex(decoder_funct3), //function type from decoder stage
        .o_funct3_exmemcsr(alu_funct3), //function type
        .i_opcode_idex(decoder_opcode), //opcode type from previous stage
        .o_opcode_exmemcsr(alu_opcode), //opcode type
        .i_exception_idex(decoder_exception), //exception from decoder stage
        .o_exception_excsr(alu_exception), //exception: illegal inst,ecall,ebreak,mret
        .o_alu_result_exmem(alu_y), //result of arithmetic operation
        // PC Control
        .i_pc_idex(decoder_pc), //pc from decoder stage
        .o_pc_exmem(alu_pc), // current pc 
        .o_next_pc(alu_next_pc), //next pc 
        .o_change_pc(alu_change_pc), //change pc if high
        // Basereg Control
        .o_wr_rd(alu_wr_rd), //write rd to basereg if enabled
        .i_rd_addr_idex(decoder_rd_addr), //address for destination register (from previous stage)
        .o_rd_addr(alu_rd_addr), //address for destination register
        .o_rd_data(alu_rd), //value to be written back to destination register
        .o_rd_valid(alu_rd_valid), //high if o_rd is valid (not load nor csr instruction)
         /// Pipeline Control ///
        .o_stall_from_alu(o_stall_from_alu), //prepare to stall next stage(memory-access stage) for load/store instruction
        .i_ce(alu_ce), // input clk enable for pipeline stalling of this stage
        .o_ce(memoryaccess_ce), // output clk enable for pipeline stalling of next stage
        .i_stall((stall_memoryaccess || stall_writeback)), //informs this stage to stall
        .i_force_stall(alu_force_stall), //force this stage to stall
        .o_stall(stall_alu), //informs pipeline to stall
        .i_flush(memoryaccess_flush), //flush this stage
        .o_flush(alu_flush) //flushes previous stages
    );
    
    asrv32_memoryaccess m4( //logic controller for data memory access (load/store) [MEMORY STAGE , STAGE 4]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_rs2_data_exmem(alu_rs2), //data to be stored to memory is always rs2
        .i_data_from_memory(i_data_from_memory), //data retrieve from memory 
        .i_result_from_alu_exmem(alu_y), //y value from ALU (address of data to memory be stored or loaded)
        .o_result_from_alu_memwb(o_store_data_addr), //y value used as data memory address
        .i_funct3_exmem(alu_funct3), //funct3 from previous stage
        .o_funct3_memwb(memoryaccess_funct3), //funct3 (byte,halfword,word)
        .i_opcode_exmem(alu_opcode), //opcode type from previous stage
        .o_opcode_memwb(memoryaccess_opcode), //opcode type
        .i_pc_exmem(alu_pc), //PC from previous stage
        .o_pc_memwb(memoryaccess_pc), //PC value
        // Basereg Control
        .i_wr_rd(alu_wr_rd), //write rd to base reg is enabled (from memoryaccess stage)
        .o_wr_rd(memoryaccess_wr_rd), //write rd to the base reg if enabled
        .i_rd_addr(alu_rd_addr), //address for destination register (from previous stage)
        .o_rd_addr(memoryaccess_rd_addr), //address for destination register
        .i_rd(alu_rd), //value to be written back to destination reg
        .o_rd(memoryaccess_rd), //value to be written back to destination register
        // Data Memory Control
        .o_stb_data(o_stb_data), //request for read/write access to data memory
        .i_ack_data(i_ack_data), //ack by data memory (high when read data is ready or when write data is already written
        .o_store_data(o_store_data), //data to be stored to memory (mask-aligned)
        .o_load_data(memoryaccess_data_load), //data to be loaded to base reg (z-or-s extended) 
        .o_wr_mask(o_wr_mask), //write mask {byte3,byte2,byte1,byte0}
        .o_wr_mem_en(memoryaccess_wr_mem), //write to data memory if enabled
         /// Pipeline Control ///
        .i_stall_from_alu(o_stall_from_alu), //stalls this stage when incoming instruction is a load/store
        .i_ce(memoryaccess_ce), // input clk enable for pipeline stalling of this stage
        .o_ce(writeback_ce), // output clk enable for pipeline stalling of next stage
        .i_stall(stall_writeback), //informs this stage to stall
        .o_stall(stall_memoryaccess), //informs pipeline to stall
        .i_flush(writeback_flush), //flush this stage
        .o_flush(memoryaccess_flush) //flushes previous stages
    );
    
    asrv32_writeback m5( //logic controller for the next PC and rd value [WRITEBACK STAGE , STAGE 5]
        .i_funct3(memoryaccess_funct3), //function type
        .i_load_data_from_memory(memoryaccess_data_load), //data to be loaded to base reg (from previous stage)
        .i_load_data_from_csr(csr_out), //CSR value to be loaded to basereg
        // .i_opcode_load(memoryaccess_opcode[`LOAD]),
        // .i_opcode_system(memoryaccess_opcode[`SYSTEM]), 
        .i_opcode(memoryaccess_opcode),
        // Basereg Control
        .i_wr_rd_en(memoryaccess_wr_rd), //write rd to base reg is enabled (from memoryaccess stage)
        .o_wr_rd_en(writeback_wr_rd), //write rd to the base reg if enabled
        .i_rd_addr(memoryaccess_rd_addr), //address for destination register (from previous stage)
        .o_rd_addr(writeback_rd_addr), //address for destination register
        .i_rd_data(memoryaccess_rd), //value to be written back to destination reg
        .o_rd_data(writeback_rd), //value to be written back to destination register
        // PC Control
        .i_pc(memoryaccess_pc), //pc value
        .o_next_pc(writeback_next_pc), //new PC value
        .o_change_pc(writeback_change_pc), //high if PC needs to jump
        // Trap-Handler
        .i_go_to_trap(csr_go_to_trap), //high before going to trap (if exception/interrupt detected)
        .i_return_from_trap(csr_return_from_trap), //high before returning from trap (via mret)
        .i_return_address(csr_return_address), //mepc CSR
        .i_trap_address(csr_trap_address), //mtvec CSR
        /// Pipeline Control ///
        .i_ce(writeback_ce), // input clk enable for pipeline stalling of this stage
        .o_stall(stall_writeback), //informs pipeline to stall
        .o_flush(writeback_flush) //flushes previous stages 
    );
    
    // removable extensions
    if(ZICSR_EXTENSION == 1) begin: zicsr
        asrv32_csr #(.TRAP_ADDRESS(TRAP_ADDRESS)) m6( // control logic for Control and Status Registers (CSR) [STAGE 4]
            .i_clk(i_clk),
            .i_rst_n(i_rst_n),
            // Interrupts
            .i_external_interrupt(i_external_interrupt), //interrupt from external source
            .i_software_interrupt(i_software_interrupt), //interrupt from software (inter-processor interrupt)
            .i_timer_interrupt(i_timer_interrupt), //interrupt from timer
            /// Exceptions ///
            .i_is_inst_illegal(alu_exception[`ILLEGAL]), //illegal instruction
            .i_is_ecall(alu_exception[`ECALL]), //ecall instruction
            .i_is_ebreak(alu_exception[`EBREAK]), //ebreak instruction
            .i_is_mret(alu_exception[`MRET]), //mret (return from trap) instruction
            /// Load/Store Misaligned Exception///
            .i_opcode(alu_opcode), //opcode type from alu stage
            .i_alu_result(alu_y), //y value from ALU (address used in load/store/jump/branch)
            /// CSR instruction ///
            .i_funct3(alu_funct3), // CSR instruction operation
            .i_csr_index(alu_imm), //immediate value decoded by decoder
            .i_imm({27'b0,alu_rs1_addr}), //unsigned immediate for immediate type of CSR instruction (new value to be stored to CSR)
            .i_rs1(alu_rs1), //Source register 1 value (new value to be stored to CSR)
            .o_csr_out(csr_out), //CSR value to be loaded to basereg
            // Trap-Handler 
            .i_pc(alu_pc), //Program Counter  (three stages had already been filled [fetch -> decode -> execute ])
            .writeback_change_pc(writeback_change_pc), //high if writeback will issue change_pc (which will override this stage)
            .o_return_address(csr_return_address), //mepc CSR
            .o_trap_address(csr_trap_address), //mtvec CSR
            .o_go_to_trap_q(csr_go_to_trap), //high before going to trap (if exception/interrupt detected)
            .o_return_from_trap_q(csr_return_from_trap), //high before returning from trap (via mret)
            .i_minstret_inc(writeback_ce), //high for one clock cycle at the end of every instruction
            /// Pipeline Control ///
            .i_ce(memoryaccess_ce), // input clk enable for pipeline stalling of this stage
            .i_stall((stall_writeback || stall_memoryaccess)) //informs this stage to stall
        );
    end
    else begin: zicsr
        assign csr_out = 0;
        assign csr_return_address = 0;
        assign csr_trap_address = 0;
        assign csr_go_to_trap = 0;
        assign csr_return_from_trap = 0;
    end
endmodule