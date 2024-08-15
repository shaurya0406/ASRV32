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

    // Trap-Handler //
    input wire[31:0] i_pc,              // Program Counter 
    input wire i_minstret_inc,          // Increment MINSTRET after executing an instruction

    /* Outputs */
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

                // TODO Machine Memory Protection

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
    
/* Internal Signals and Registers to hold input values */  

    wire opcode_store   = i_opcode[`STORE];
    wire opcode_load    = i_opcode[`LOAD];
    wire opcode_branch  = i_opcode[`BRANCH];
    wire opcode_jal     = i_opcode[`JAL];
    wire opcode_jalr    = i_opcode[`JALR];
    wire opcode_system  = i_opcode[`SYSTEM];

endmodule