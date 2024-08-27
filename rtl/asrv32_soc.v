`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

// Complete SoC package containing the main CPU asrv32_core and Main Memory (Combined Instruction & Data Memory)
module asrv32_soc #(parameter CLK_FREQ_MHZ=12, PC_RESET=32'h00_00_00_00, TRAP_ADDRESS=32'h00_00_00_00, ZICSR_EXTENSION=1, MEMORY_DEPTH=1024) ( 
    input wire i_clk, // System clock
    input wire i_rst_n, // Active low reset

    // ! Moved to VIC Module
    // //Interrupts
    // input wire i_external_interrupt, //interrupt from external source
    // input wire i_software_interrupt, //interrupt from software
    // // Timer Interrupt
    // input wire i_mtime_wr, //write to mtime
    // input wire i_mtimecmp_wr,  //write to mtimecmp
    // input wire[63:0] i_mtime_din, //data to be written to mtime
    // input wire[63:0] i_mtimecmp_din //data to be written to mtimecmp
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
    wire i_stb_data;
    wire o_ack_data;

    // Interrupts
    wire i_external_interrupt = 0; // interrupt from external source
    wire o_timer_interrupt; // interrupt from VIC
    wire o_software_interrupt; // interrupt from VIC

    // Main ASRV32 core instantiation
    asrv32_core #(.PC_RESET(PC_RESET),.CLK_FREQ_MHZ(CLK_FREQ_MHZ), .TRAP_ADDRESS(TRAP_ADDRESS)) m0( 
        .i_clk(i_clk), // System clock
        .i_rst_n(i_rst_n), // Active low reset
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
        .i_clk(i_clk), // System clock
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
    output reg[31:0] o_inst_out, // Instruction output
    input wire i_stb_inst, // request for instruction
    output reg o_ack_inst, //ack (high if new instruction is now on the bus)
    // Data Memory Interface
    input wire[$clog2(MEMORY_DEPTH)-1:0] i_data_addr, // Data address
    input wire[31:0] i_data_in, // Data input
    input wire[3:0] i_wr_mask, // Write mask
    input wire i_wr_en, // Write enable
    input wire i_stb_data,
    output reg o_ack_data,
    output reg[31:0] o_data_out // Data output
);
    // Memory array for storing instructions and data
    reg[31:0] memory_regfile[MEMORY_DEPTH/4 - 1:0];
    integer i;
    
    // Initialize memory to zero
    initial begin 
        // for(i = 0; i < MEMORY_DEPTH/4 - 1; i = i + 1) 
        //     memory_regfile[i] = 0; 
        $readmemh("memory.mem",memory_regfile);
        o_ack_inst <= 0;
        o_ack_data <= 0;
        o_inst_out <= 0;
    end
    
    //reading must be registered to be inferred as block ram
    always @(posedge i_clk) begin 
        // Inst Acknowledge sent in the next clock cycle
        o_ack_inst <= i_stb_inst; 
        // Read instruction from memory, sent out in next clock cycle
        o_inst_out <= memory_regfile[{i_inst_addr >> 2}]; 
        // Data Acknowledge sent in the next clock cycle
        o_ack_data <= i_stb_data;
        // Read data from memory, sent out in next clock cycle
        o_data_out <= memory_regfile[i_data_addr[$clog2(MEMORY_DEPTH)-1:2]]; 
    end

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



module asrv32_vic #( // Vectored Interrupt Controller
    parameter CLK_FREQ_MHZ = 12, //input clock frequency in MHz

    // * A MTIMER device has two separate base addresses: one for the MTIME register and another for the MTIMECMP registers. 
    parameter MTIME_BASE_ADDRESS = 8008,
              MTIMECMP_BASE_ADDRESS = 8016,
              MSIP_BASE_ADDRESS = 8024
)(
        input wire clk,
        input wire rst_n,
        input wire[31:0] clint_address, // read/write address (access the memory-mapped registers for controlling i2c)
        input wire[31:0] clint_wdata, // data to be written to slave or to memory-mapped registers of i2c
        output reg[31:0] clint_rdata, // data retrieved from slave or from the memory-mapped registers of i2c
        input wire clint_wr_en, // write-enable
        input wire i_stb, // request to access CLINT
        output reg o_ack, // acknowledge by CLINT

        // Interrupts
        output wire o_timer_interrupt,
        output wire o_software_interrupt
);
    // ? This is based from RISC-V Advanced Core Local Interruptor
    // ? Specification: https://github.com/riscv/riscv-aclint/blob/main/riscv-aclint.adoc

    // ? This RISC-V ACLINT specification defines a set of memory mapped devices which provide 
    // ? inter-processor interrupts (IPI) and timer functionalities.
    // ? The MTIMER device provides machine-level timer functionality for a set of HARTs on a RISC-V platform. 
    // ? It has a single fixed-frequency monotonic time counter (MTIME) register and a time 
    // ? compare register (MTIMECMP) for each HART connected to the MTIMER device.
    
    reg[63:0] mtime = 0;
    reg[63:0] mtimecmp = {64{1'b1}};   
    reg msip = 0; //Inter-processor (or software) interrupts


   //READ memory-mapped registers 
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            o_ack <= 0;
            clint_rdata <= 0;
        end
        else begin
            if(i_stb && !clint_wr_en) begin //read the memory-mapped register
                if(clint_address == MTIME_BASE_ADDRESS) clint_rdata <= mtime[31:0]; //first half 
                else if(clint_address == MTIME_BASE_ADDRESS + 4) clint_rdata <= mtime[63:32]; //second half
                if(clint_address == MTIMECMP_BASE_ADDRESS) clint_rdata <= mtimecmp[31:0]; //first half
                else if(clint_address == MTIMECMP_BASE_ADDRESS + 4) clint_rdata <= mtimecmp[63:32]; //second half
                if(clint_address == MSIP_BASE_ADDRESS) clint_rdata <= {31'b0, msip}; //machine software interrupt
            end
            o_ack <= i_stb; //wishbone protocol stb-ack mechanism
        end
    end

    //WRITE to memory-mapped registers 
    always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            mtime <= 64'd0;
            mtimecmp <= {64{1'b1}}; //timer interrupt will be triggered unintentionally if reset at 0 (equal to mtime) 
                                  //thus we set it at highest value (all 1s)
            msip <= 0;
        end
        else begin
            if(i_stb && clint_wr_en) begin //write to the memory-mapped registers
                if(clint_address == MTIME_BASE_ADDRESS)  mtime[31:0] <= clint_wdata; //first half 
                else if(clint_address == MTIME_BASE_ADDRESS + 4) mtime[63:32] <= clint_wdata; //second half
                if(clint_address == MTIMECMP_BASE_ADDRESS) mtimecmp[31:0] <= clint_wdata; //first half
                else if(clint_address == MTIMECMP_BASE_ADDRESS + 4) mtimecmp[63:32] <= clint_wdata; //second half
                if(clint_address == MSIP_BASE_ADDRESS) msip <= clint_wdata[0]; //machine software interrupt
            end
            mtime <= mtime + 1'b1; //increment every clock tick (so timer freq is same as cpu clock freq)
        end
    end

    //? Volume 2 pg. 44: Platforms provide a 64-bit memory-mapped machine-mode timer compare register (mtimecmp). 
    //? A machine timer interrupt becomes pending whenever mtime contains a value greater than or equal to mtimecmp, 
    //? treating the values as unsigned integers. The interrupt remains posted until mtimecmp becomes greater than
    //? mtime (typically as a result of writing mtimecmp). 
    
    assign o_timer_interrupt = (mtime >= mtimecmp);

    //? Each MSIP register is a 32-bit wide WARL register where the upper 31 bits are wired to zero.
    //? The least significant bit is reflected in MSIP of the mip CSR. A machine-level software interrupt 
    //? for a HART is pending or cleared by writing 1 or 0 respectively to the corresponding MSIP register.
    
     assign o_software_interrupt = msip;

endmodule



module peripheral_bus_controller ( // Decodes address and access the corresponding memory-mapped device
    // ASRV32 Core
    input wire[31:0] i_data_addr,
    input wire[31:0] i_wdata,
    output reg[31:0] o_rdata,
    input wire i_wr_en,
    input wire[3:0] i_wr_mask,
    input wire i_stb_data,
    output reg o_ack_data,

    // Device 0 : RAM
    output reg[31:0] o_device0_data_addr,
    output reg[31:0] o_device0_wdata,
    input wire[31:0] i_device0_rdata,
    output reg o_device0_wr_en,
    output reg[3:0] o_device0_wr_mask,
    output reg o_device0_stb_data,
    input wire i_device0_ack_data,

    // Device 1: VIC
    output reg[31:0] o_device1_data_addr,
    output reg[31:0] o_device1_wdata,
    input wire[31:0] i_device1_rdata,
    output reg o_device1_wr_en,
    output reg[3:0] o_device1_wr_mask,
    output reg o_device1_stb_data,
    input wire i_device1_ack_data
)
    always @(*) begin
        o_rdata = 0;
        o_ack_data = 0;

        o_device0_data_addr = 0; 
        o_device0_wdata = 0;
        o_device0_wr_en = 0;
        o_device0_wr_mask = 0;
        o_device0_stb_data = 0;

        o_device1_data_addr = 0; 
        o_device1_wdata = 0;
        o_device1_wr_en = 0;
        o_device1_wr_mask = 0;
        o_device1_stb_data = 0;

        // Memory Mapped peripherals address have MSB set to 1
        if(i_data_addr[31]) begin
            if(i_data_addr[11:0] < 12'h50) begin // Device 1 Interface (VIC) (20 words)
                o_device1_data_addr = i_data_addr; 
                o_device1_wdata = i_wdata;
                o_rdata = i_device1_rdata;
                o_device1_wr_en = i_wr_en;
                o_device1_wr_mask = i_wr_mask;
                o_device1_stb_data = i_stb_data;
                o_ack_data = i_device1_ack_data;
            end
        end
        else begin
            o_device0_data_addr = i_data_addr; 
            o_device0_wdata = i_wdata;
            o_rdata = i_device0_rdata;
            o_device0_wr_en = i_wr_en;
            o_device0_wr_mask = i_wr_mask;
            o_device0_stb_data = i_stb_data;
            o_ack_data = i_device0_ack_data;
        end
    end   
endmodule