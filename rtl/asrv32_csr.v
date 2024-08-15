`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* Decoder Module & Ports Declaration*/
module asrv32_csr #(parameter CLK_FREQ_MHZ = 100, TRAP_ADDRESS = 0) (

    /* Inputs */
    input wire i_clk, i_rst_n,
    input wire i_csr_stage_en, // Enable csr read/write iff stage is currently on MEMORYACCESS

    // Interrupts //
    input wire i_external_interrupt, // External Interrupt
    input wire i_software_interrupt, // Internal Software Interrupt

    // TODO: Memory Mapped Timer 
    // Timer Interrupt //
    input wire i_mtime_wr_en,           // Write Enable for MTIME Register
    input wire i_mtimecmp_wr_en,        // Write Enable for MTIMECMP Register
    input wire[63:0] i_mtime_din,       // Data to be written to mtime
    input wire[63:0] i_mtimecmp_din,    // Data to be written to mtimecmp

    // Exceptions //
    input wire i_is_inst_illegal,   // ILLEGAL instruction
    input wire i_is_ecall,          // ECALL instruction
    input wire i_is_ebreak,         // EBREAK instruction
    input wire i_is_mret,           // MRET (return from trap) instruction

    // Instruction/Load/Store Misaligned Exception //
    input wire[`OPCODE_WIDTH-1:0] i_opcode, // Opcode types required: LOAD, STORE, BRANCH, JAL & JALR
    input wire[31:0] i_alu_result,          // Sum from ALU (address used in load/store/jump/branch)

    // CSR Instruction // 
    input wire[2:0]  i_funct3,      // CSR instruction operation (Opcode Type Required: SYSTEM)
    input wire[11:0] i_csr_index,   // Immediate value from decoder
    input wire[31:0] i_imm,         // Unsigned Immediate for immediate type of CSR instruction (new value to be stored to CSR)
    input wire[31:0] i_rs1,         // Source register 1 value (new value to be stored to CSR)
    
    output reg[31:0] o_csr_out, //CSR value to be loaded to basereg
    
    // Trap-Handler //
    input wire[31:0] i_pc,              // Program Counter 
    input wire i_minstret_inc,          // Increment MINSTRET after executing an instruction

    output reg[31:0] o_return_address,  // MEPC CSR
    output reg[31:0] o_trap_address,    // MTVEC CSR
    output reg o_go_to_trap_q,          // High before going to trap (if exception/interrupt detected)
    output reg o_return_from_trap_q     // High before returning from trap (via mret)
    
);

/* CSR operation type */
    localparam  CSRRW    = 3'b001,
                CSRRS    = 3'b010,
                CSRRC    = 3'b011,
                CSRRWI   = 3'b101,
                CSRRSI   = 3'b110,
                CSRRCI   = 3'b111;
               
/* CSR Register Addresses */          
    localparam  // Machine Info
                MVENDORID    = 12'hF11, // Vendor ID.
                MARCHID      = 12'hF12, // Architecture ID.
                MIMPID       = 12'hF13, // Implementation ID.
                MHARTID      = 12'hF14, // Hardware thread ID.

                // Machine Trap Setup
                MSTATUS  = 12'h300, // Machine status register.
                MISA     = 12'h301, // ISA and extensions
                MIE      = 12'h304, // Machine interrupt-enable register.
                MTVEC    = 12'h305, // Machine trap-handler base address.

                // Machine Trap Handling
                MSCRATCH = 12'h340, // Scratch register for machine trap handlers.
                MEPC     = 12'h341, // Machine exception program counter.
                MCAUSE   = 12'h342, // Machine trap cause.
                MTVAL    = 12'h343, // Machine bad address or instruction.
                MIP      = 12'h344, // Machine interrupt pending.

                // TODO Machine Memory Protection (Optional)

                // Machine Counter/Timers
                MCYCLE        = 12'hB00, // Machine cycle counter.
                MCYCLEH       = 12'hB80, // Upper 32 bits of mcycle, RV32 only.

                MINSTRET      = 12'hB02, // Machine instructions-retired counter.
                MINSTRETH     = 12'hBB2, // Upper 32 bits of minstret, RV32 only.

                //Machine Counter Setup
                MCOUNTINHIBIT = 12'h320, // Machine counter-inhibit register.

                // Unprivileged Counter/Timers
                TIME     = 12'hC01, // Timer for RDTIME instruction.
                TIMEH    = 12'hC81; // Upper 32 bits of time, RV32 only.

/* MCAUSE Codes */
    localparam  // Interrupt Codes
                MACHINE_SOFTWARE_INTERRUPT      = 3,
                MACHINE_TIMER_INTERRUPT         = 7,
                MACHINE_EXTERNAL_INTERRUPT      = 11,

                // Trap Codes
                INSTRUCTION_ADDRESS_MISALIGNED  = 0,
                ILLEGAL_INSTRUCTION             = 2,
                EBREAK                          = 3,
                LOAD_ADDRESS_MISALIGNED         = 4,
                STORE_ADDRESS_MISALIGNED        = 6,
                ECALL                           = 11;

/* Wrap value for 1 millisecond */ 
    localparam MILLISEC_WRAP =  (CLK_FREQ_MHZ*10**6)/1000;

// Initialise Outputs to 0 For Testbench
initial begin
        o_csr_out = 0;
        o_return_address = 0;
        o_trap_address = 0;
        o_go_to_trap_q = 0;
        o_return_from_trap_q = 0;
    end

/* Internal Signals and Registers to hold input values */  

    wire opcode_store   = i_opcode[`STORE];
    wire opcode_load    = i_opcode[`LOAD];
    wire opcode_branch  = i_opcode[`BRANCH];
    wire opcode_jal     = i_opcode[`JAL];
    wire opcode_jalr    = i_opcode[`JALR];
    wire opcode_system  = i_opcode[`SYSTEM];

    wire csr_enable = opcode_system && i_funct3!=0 && i_csr_stage_en; // CSR operation is enabled only at this conditions
    reg[31:0] csr_in;   // Data to be stored to CSR
    reg[31:0] csr_data; // Data at current CSR address

    reg[1:0] new_pc = 0; // Last two bits of i_pc that will be used in taken branch and jumps

    reg is_trap;
    reg go_to_trap;         // High before going to trap (if exception/interrupt detected)
    reg return_from_trap;   // High before returning from trap (via mret)

    reg is_exception;
    reg is_load_addr_misaligned; 
    reg is_store_addr_misaligned;
    reg is_inst_addr_misaligned;

    reg is_interrupt;
    reg timer_interrupt;
    reg external_interrupt_pending; 
    reg software_interrupt_pending;
    reg timer_interrupt_pending;

/* Initialise CSR Individual Register Bits */

    reg mstatus_mie = 0;            // Machine Interrupt Enable
    reg mstatus_mpie = 0;           // Machine Previous Interrupt Enable
    reg[1:0] mstatus_mpp = 2'b11;   // MPP

    reg mie_meie = 0;   // Machine external interrupt enable
    reg mie_mtie = 0;   // Machine timer interrupt enable
    reg mie_msie = 0;   // Machine software interrupt enable

    reg[29:0] mtvec_base = TRAP_ADDRESS[31:2];  // Address of i_pc after returning from interrupt (via MRET)
    reg[1:0] mtvec_mode = TRAP_ADDRESS[1:0];    // Vector mode addressing 

    reg[31:0] mscratch = 0; // Dedicated for use by machine code
    reg[31:0] mepc = 0;     // Machine exception i_pc (address of interrupted instruction)
    
    reg mcause_intbit = 0;      // Interrupt(1) or Exception(0)
    reg[3:0] mcause_code = 0;   // Indicates event that caused the trap
    
    reg[31:0] mtval = 0; // Exception-specific infotmation to assist software in handling trap
    
    reg mip_meip = 0; // Machine external interrupt pending
    reg mip_mtip = 0; // Machine timer interrupt pending
    reg mip_msip = 0; // Machine software interrupt pending
    
    reg[63:0] mtime = 0;                            // Real-time i_clk (millisecond increment)
    reg[$clog2(MILLISEC_WRAP)-1:0] millisec = 0;    // Counter with period of 1 millisec
    reg[63:0] mtimecmp = 0;                         // Compare register for mtime
    
    reg[63:0] mcycle = 0;   // Counts number of i_clk cycle executed by core
    reg[63:0] minstret = 0; // Counts number instructions retired/executed by core
    
    reg mcountinhibit_cy = 0; // Controls increment of mcycle
    reg mcountinhibit_ir = 0; // Controls increment of minstret

/* Combinational Logic for load/store/instruction misaligned exception detection */

    always @* begin
        is_load_addr_misaligned = 0;
        is_store_addr_misaligned = 0;
        is_inst_addr_misaligned = 0;
        new_pc = 0;
        
        // Misaligned Load/Store Address
        if(i_funct3[1:0] == 2'b01) begin // halfword load/store
            is_load_addr_misaligned = opcode_load? i_alu_result[0] : 0;
            is_store_addr_misaligned = opcode_store? i_alu_result[0] : 0;
        end
        if(i_funct3[1:0] == 2'b10) begin // word load/store
            is_load_addr_misaligned = opcode_load? i_alu_result[1:0]!=2'b00 : 0;
            is_store_addr_misaligned = opcode_store? i_alu_result[1:0]!=2'b00 : 0;
        end
        
        // Misaligned Instruction Address
        /* 
        ? Volume 1 pg. 15: Instructions are 32 bits in length and must be aligned on a four-byte boundary in memory.
        ? An instruction-address-misaligned exception is generated on a taken branch or unconditional jump
        ? if the target address is not four-byte aligned. This exception is reported on the branch or jump
        ? instruction, not on the target instruction. No instruction-address-misaligned exception is generated
        ? for a conditional branch that is not taken. 
        */

        if((opcode_branch && i_alu_result[0]) || opcode_jal || opcode_jalr) begin // branch or jump to new instruction
            new_pc = i_pc[1:0] + i_csr_index[1:0];
            if(opcode_jalr) new_pc = i_rs1[1:0] +  i_csr_index[1:0];
            is_inst_addr_misaligned = (new_pc == 2'b00)? 1'b0:1'b1; //i_pc (instruction address) must always be four bytes aligned
        end     
    end

/* Cobinational Logic for trap-handling sequence */
    always @(*) begin
        // Resetting all values for initialisation
        external_interrupt_pending = 0;
        software_interrupt_pending = 0;
        timer_interrupt_pending = 0;
        is_interrupt = 0;
        is_exception = 0;
        is_trap = 0;
        go_to_trap = 0;
        return_from_trap = 0;

        // Assigning Updated Values
        if(i_csr_stage_en) begin
            external_interrupt_pending =  mstatus_mie && mie_meie && (mip_meip);   // machine_interrupt_enable + machine_external_interrupt_enable + machine_external_interrupt_pending must all be high
            software_interrupt_pending = mstatus_mie && mie_msie && mip_msip;      // machine_interrupt_enable + machine_software_interrupt_enable + machine_software_interrupt_pending must all be high
            timer_interrupt_pending = mstatus_mie && mie_mtie && mip_mtip;         // machine_interrupt_enable + machine_timer_interrupt_enable + machine_timer_interrupt_pending must all be high
            
            is_interrupt = external_interrupt_pending || software_interrupt_pending || timer_interrupt_pending;
            is_exception = i_is_inst_illegal || is_inst_addr_misaligned || i_is_ecall || i_is_ebreak || is_load_addr_misaligned || is_store_addr_misaligned ;
            is_trap = is_interrupt || is_exception;
            go_to_trap = is_trap; // A trap is taken, save i_pc, and go to trap address
            return_from_trap = i_is_mret; // Return from trap, go back to saved i_pc
        end
    end

/* Assign CSR Stored Data based on regsters (Combinational Logic) */

    always @(*) begin
        csr_data = 0;
        csr_in = 0;

        case(i_csr_index)
            // Machine info
            MVENDORID   :       csr_data = 32'h0; // MVENDORID (JEDEC manufacturer ID)
            MARCHID     :       csr_data = 32'h0; // MARCHID (open-source project architecture ID allocated by RISC-V International  ( https://github.com/riscv/riscv-isa-manual/blob/master/marchid.md ))
            MIMPID      :       csr_data = 32'h0; // MIMPID (version of the processor implementation (provided by author of source code))
            MHARTID     :       csr_data = 32'h0; // MHARTID (integer ID of the hart that is currently running the code (one hart must have an ID of zero))             
            
            // Machine trap setup  
            MSTATUS:        begin // MSTATUS (controls hart's current operating state (mie and mpie are the only configurable bits))
                                csr_data[3] = mstatus_mie;
                                csr_data[7] = mstatus_mpie;
                                csr_data[12:11] = mstatus_mpp; //MPP
                            end

            MISA:           begin // MISA (control and monitor hart's current operating state)
                                csr_data[8] = 1'b1;         // RV32I/64I/128I base ISA (ISA supported by the hart)
                                csr_data[31:30] = 2'b01;    // Base 32
                            end

            MIE:            begin // MIE (interrupt enable bits)
                                csr_data[3] = mie_msie;
                                csr_data[7] = mie_mtie;
                                csr_data[11] = mie_meie;
                            end

            MTVEC:          begin // MTVEC (trap vector configuration (base+mode))
                                csr_data = {mtvec_base,mtvec_mode};
                            end
                       
            // Machine trap handling
            MSCRATCH:       begin // MSCRATCH (dedicated for use by machine code) 
                                csr_data = mscratch;
                            end

            MEPC:           begin // MEPC (address of interrupted instruction)
                                csr_data = mepc; 
                            end

            MCAUSE:         begin // MCAUSE (indicates cause of trap(either interrupt or exception))
                                csr_data[31] = mcause_intbit; 
                                csr_data[3:0] = mcause_code;
                            end

            MTVAL:          begin // MTVAL (exception-specific information to assist software in handling trap)
                                csr_data = mtval;
                            end

            MIP:            begin // MIP (pending interrupts)
                                csr_data[3] = mip_msip;
                                csr_data[7] = mip_mtip;
                                csr_data[11] = mip_meip;
                            end
                       
            // Machine counters/timers
            MCYCLE:         begin // MCYCLE (counts number of i_clk cycle executed by core [LOWER HALF])
                                csr_data = mcycle[31:0];
                            end

            MCYCLEH:        begin // MCYCLE (counts number of i_clk cycle executed by core [UPPER HALF])
                                csr_data = mcycle[63:32];
                            end

            TIME:           begin // TIME (real-time i_clk [millisecond increment] [LOWER HALF])
                                csr_data = mtime[31:0];  
                            end

            TIMEH:          begin // TIME (real-time i_clk [millisecond increment] [LOWER HALF])
                                csr_data = mtime[63:32]; 
                            end

            MINSTRET:       begin // MINSTRET (counts number instructions retired/executed by core [LOWER half])     
                                csr_data = minstret[31:0];
                            end

            MINSTRETH:      begin // MINSTRET (counts number instructions retired/executed by core [UPPER half])     
                                csr_data = minstret[63:32];
                            end
                       
            MCOUNTINHIBIT:  begin // MCOUNTINHIBIT (controls which hardware performance-monitoring counters can increment)
                                csr_data[0] = mcountinhibit_cy;
                                csr_data[2] = mcountinhibit_ir;
                            end
                       
            default: csr_data = 0;

        endcase
    end

/* Assign CSR Input Data to be stored (Combinational Logic) */
    always @(*) begin
        // Assign csr_in (data TO BE stored to CSR)
        case(i_funct3) // CSR instruction type
            CSRRW: csr_in = i_rs1;                  // CSR read-write
            CSRRS: csr_in = csr_data | i_rs1;       // CSR read-set
            CSRRC: csr_in = csr_data & (~i_rs1);    // CSR read-clear
           CSRRWI: csr_in = i_imm;                  // csr read-write immediate
           CSRRSI: csr_in = csr_data | i_imm;       // CSR read-set immediate
           CSRRCI: csr_in = csr_data & (~i_imm);    // CSR read-clear immediate
        endcase
    end

/* 
TODO: Control Logic for writing to CSRs
   * CSR Control Logic
   * Register the Outputs
*/

endmodule