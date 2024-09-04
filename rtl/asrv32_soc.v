`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

// Complete SoC package containing the main CPU asrv32_core and Main Memory (Combined Instruction & Data Memory)
module asrv32_soc #(parameter CLK_FREQ_MHZ=12, PC_RESET=32'h00_00_00_00, TRAP_ADDRESS=32'h00_00_00_00, ZICSR_EXTENSION=1, MEMORY_DEPTH=1024, GPIO_COUNT = 4) ( 
    input wire i_clk, // System clock
    input wire i_rst // Active low reset
    //UART
    input wire uart_rx,
    output wire uart_tx,
    //I2C
    inout wire i2c_sda,
    inout wire i2c_scl,
    //GPIO
    inout wire[GPIO_COUNT-1:0] gpio_pins
    );
    
    // Instruction Memory Interface
    wire[31:0] inst; // Instruction from memory
    wire[31:0] iaddr; // Instruction address from CPU
    wire i_stb_inst;
    wire o_ack_inst;
    
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

    // Peripheral Bus Controller
    wire[31:0] o_device0_data_addr;
    wire[31:0] o_device0_wdata;
    wire[31:0] i_device0_rdata;
    wire o_device0_wr_en;
    wire[3:0]  o_device0_wr_mask;
    wire o_device0_stb_data;
    wire i_device0_ack_data;

    wire[31:0] o_device1_data_addr;
    wire[31:0] o_device1_wdata;
    wire[31:0] i_device1_rdata;
    wire o_device1_wr_en;
    wire[3:0] o_device1_wr_mask;
    wire o_device1_stb_data;
    wire i_device1_ack_data;

// Main ASRV32 core instantiation
    asrv32_core #(.PC_RESET(PC_RESET), .TRAP_ADDRESS(TRAP_ADDRESS)) m0( 
        .i_clk(i_clk), // System clock
        .i_rst_n(!i_rst), // Active low reset

        // Instruction Memory Interface
        .i_inst(inst), // 32-bit instruction
        .o_inst_addr(iaddr), // Address of instruction 
        .o_stb_inst(i_stb_inst), // request for read access to instruction memory
        .i_ack_inst(o_ack_inst),  // ack (high if new instruction is ready)

        // Data Memory Interface
        .i_data_from_memory(din), // Data retrieved from memory
        .o_store_data(dout), // Data to be stored to memory
        .o_store_data_addr(daddr), // Address of data memory for store/load
        .o_wr_mask(wr_mask), // Write mask control
        .o_wr_en(wr_en), // Write enable 
        .o_stb_data(i_stb_data), //request for read/write access to data memory
        .i_ack_data(o_ack_data), //ack by data memory (high when read data is ready or when write data is already written)

        //Interrupts
        .i_external_interrupt(i_external_interrupt), //interrupt from external source
        .i_software_interrupt(o_software_interrupt), //interrupt from software
        .i_timer_interrupt(o_timer_interrupt) //interrupt from timer
    );

// Peripheral Bus Controller Instantiation
    peripheral_bus_controller controller(
        // ARV32 Core
        .i_data_addr(daddr),
        .i_wdata(dout),
        .o_rdata(din),
        .i_wr_en(wr_en),
        .i_wr_mask(wr_mask),
        .i_stb_data(i_stb_data),
        .o_ack_data(o_ack_data),

        //Device 0 Interface (RAM)
        .o_device0_data_addr(o_device0_data_addr),
        .o_device0_wdata(o_device0_wdata),
        .i_device0_rdata(i_device0_rdata),
        .o_device0_wr_en(o_device0_wr_en),
        .o_device0_wr_mask(o_device0_wr_mask),
        .o_device0_stb_data(o_device0_stb_data),
        .i_device0_ack_data(i_device0_ack_data),

        //Device 1 Interface (CLINT)
        .o_device1_data_addr(o_device1_data_addr),
        .o_device1_wdata(o_device1_wdata),
        .i_device1_rdata(i_device1_rdata),
        .o_device1_wr_en(o_device1_wr_en),
        .o_device1_wr_mask(o_device1_wr_mask),
        .o_device1_stb_data(o_device1_stb_data),
        .i_device1_ack_data(i_device1_ack_data)
    );


// Device 0: Main memory instantiation
    main_memory #(.MEMORY_DEPTH(MEMORY_DEPTH)) m1(
        .i_clk(i_clk), // System clock
        // Instruction Memory Interface
        .i_inst_addr(iaddr[$clog2(MEMORY_DEPTH)-1:0]), // Instruction address
        .o_inst_out(inst), // Instruction output
        .i_stb_inst(i_stb_inst), 
        .o_ack_inst(o_ack_inst), 
        // Data Memory Interface
        .i_data_addr(o_device0_data_addr[$clog2(MEMORY_DEPTH)-1:0]), // Data address
        .i_data_in(o_device0_wdata), // Data input
        .i_wr_mask(o_device0_wr_mask), // Write mask
        .i_wr_en(o_device0_wr_en), // Write enable
        .i_stb_data(o_device0_stb_data),
        .o_ack_data(i_device0_ack_data),
        .o_data_out(i_device0_rdata) // Data output
    );

// Device 1: VIC instantiation
    asrv32_vic #( //Core Logic Interrupt [memory-mapped to < h50 (MSB=1)]
        .CLK_FREQ_MHZ(CLK_FREQ_MHZ), //input clock frequency in MHz
        .MTIME_BASE_ADDRESS(32'h8000_0000),  //Machine-level timer register (64-bits, 2 words)
        .MTIMECMP_BASE_ADDRESS(32'h8000_0008), //Machine-level Time Compare register (64-bits, 2 words)
        .MSIP_BASE_ADDRESS(32'h8000_0010) //Machine-level Software Interrupt register
    ) vic  (
        .clk(i_clk),
        .rst_n(!i_rst),
        .clint_address(o_device1_data_addr), //read/write address (access the memory-mapped registers for controlling i2c)
        .clint_wdata(o_device1_wdata), //data to be written to slave or to memory-mapped registers of i2c
        .clint_rdata(i_device1_rdata), //data retrieved from slave or from the memory-mapped registers of i2c
        .clint_wr_en(o_device1_wr_en), //write-enable
        .i_stb(o_device1_stb_data), //request to access CLINT
        .o_ack(i_device1_ack_data), //acknowledge by CLINT
        // Interrupts
        .o_timer_interrupt(o_timer_interrupt),
        .o_software_interrupt(o_software_interrupt)
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
);
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



module uart #( //UART (TX only)
    parameter CLOCK_FREQ = 12_000_000,//Input clock frequency
    parameter BAUD_RATE  = 9600, //UART Baud rate
    parameter UART_TX_DATA = 8140, //memory-mapped address for TX (write to UART)
    parameter UART_TX_BUSY = 8144, //memory-mapped address to check if TX is busy (has ongoing request)
    parameter UART_RX_BUFFER_FULL = 8148, //memory-mapped address  to check if a read has completed
    parameter UART_RX_DATA = 8152, //memory-mapped address for RX (read the data)
    parameter DBIT = 8, //UART Data Bits
    parameter SBIT = 1 //UART Stop Bits
    )(
        input wire clk,
        input wire rst_n,
        input wire[31:0] uart_rw_address, //read/write address (access the memory-mapped registers for controlling UART)
        input wire[DBIT - 1:0 ] uart_wdata, //TX data
        input wire uart_wr_en, //write-enable
        input wire uart_rx, //UART RX line
        output wire uart_tx, //UART TX line
        output reg[DBIT - 1:0] uart_rdata, //data read from memory-mapped register 
        input wire i_stb_data, //request to access UART
        output reg o_ack_data //acknowledge by UART
    );


    localparam DVSR = CLOCK_FREQ/(16*BAUD_RATE);
    localparam DVSR_WIDTH = $clog2(DVSR); //array size needed by DVSR
    localparam SB_TICK = 16*SBIT;
    
     //FSM state declarations
     localparam[1:0] idle=2'd0,
                    start=2'd1,
                    data=2'd2,
                    stop=2'd3;
                    
    reg[DBIT - 1:0] uart_busy;    
    reg tx_done_tick;          
    reg[1:0] state_reg,state_nxt;
    reg[3:0] s_reg,s_nxt; //count to 16 for every data bit
    reg[2:0] n_reg,n_nxt; //count the number of data bits already transmitted
    reg[DBIT - 1:0] din_reg,din_nxt; //stores the word to be transmitted
    reg tx_reg,tx_nxt;
    reg s_tick;
    reg wr_uart;
    reg[1:0] state_reg_rx,state_nxt_rx;
    reg[3:0] s_reg_rx,s_nxt_rx; //check if number of ticks is 7(middle of start bit), or 15(middle of a data bit)
    reg[2:0] n_reg_rx,n_nxt_rx; //checks how many data bits is already passed(value is 7 for last bit)
    reg[7:0] b_reg,b_nxt; //stores 8-bit binary value of received data bits
    reg[7:0] dout; //data read from UART
    reg rx_done_tick; //goes high if a read is done
    reg rx_buffer_full; //goes high if a read is done


    //baud tick generator
     reg[DVSR_WIDTH-1:0] counter=0;
     always @(posedge clk,negedge rst_n) begin
        if(!rst_n) counter<=0;
        else begin
            s_tick=0;
            if(counter == DVSR-1) begin
                s_tick=1;
                counter<=0;
            end
            else begin
                counter<=counter+1;
            end
            
        end
     end
     //Read memory-mapped registers
     always @(posedge clk, negedge rst_n) begin
        if(!rst_n) begin
            uart_rdata <= 0;
            o_ack_data <= 0;
        end
        else begin
            if(i_stb_data && !uart_wr_en && uart_rw_address == UART_TX_BUSY) begin //read request to UART_TX_BUSY_ADDR (check if there is an ongoing request)
                uart_rdata <= uart_busy;
            end
            else if(i_stb_data && !uart_wr_en && uart_rw_address == UART_RX_BUFFER_FULL) begin //read request to UART_RX_BUFFER_FULL (check if a read is completed)
                uart_rdata <= rx_buffer_full;
            end
            else if(i_stb_data && !uart_wr_en && uart_rw_address == UART_RX_DATA) begin //read request to UART_RX_DATA (read the data)
                uart_rdata <= dout;
            end
            o_ack_data <= i_stb_data;
        end
     end
     
     
     /******************************** UART TX ****************************************/

    
     //FSM register operation
     always @(posedge clk,negedge rst_n) begin
        if(!rst_n) begin
            state_reg<=idle;
            s_reg<=0;
            n_reg<=0;
            din_reg<=0;
            tx_reg<=0;
        end
        else begin
            state_reg<=state_nxt;
            s_reg<=s_nxt;
            n_reg<=n_nxt;
            din_reg<=din_nxt;
            tx_reg<=tx_nxt;
        end
     end
     
     //FSM next-state logic
     always @* begin
        state_nxt=state_reg;
        s_nxt=s_reg;
        n_nxt=n_reg;
        din_nxt=din_reg;
        tx_nxt=tx_reg;
        tx_done_tick=0; 
        uart_busy= 1; //uart is busy unless its in idle state
         case(state_reg)
                idle: begin 
                            tx_nxt=1;
                            uart_busy = 0;
                            //start transmit operation when there is a write request to UART_TX_DATA_ADDR and we are in idle
                            if(uart_wr_en && i_stb_data && uart_rw_address == UART_TX_DATA && !uart_busy) begin 
                                din_nxt=uart_wdata;
                                s_nxt=0;
                                state_nxt=start;
                                uart_busy = 1;
                            end
                        end
              start: begin   //wait to finish the start bit
                            tx_nxt=0;
                            if(s_tick==1) begin
                                if(s_reg==15) begin
                                    s_nxt=0;
                                    n_nxt=0;
                                    state_nxt=data;
                                end
                                else s_nxt=s_reg+1;
                            end
                        end
                data: begin  //wait for all data bits to be transmitted serially
                            tx_nxt=din_reg[0];
                            if(s_tick==1) begin
                                if(s_reg==15) begin
                                    din_nxt=din_reg>>1;
                                    s_nxt=0;
                                    if(n_reg==DBIT-1) state_nxt=stop;
                                    else n_nxt=n_reg+1;
                                end
                                else s_nxt=s_reg+1;
                            end
                        end
                stop: begin  //wait to finish the stop bit 
                            tx_nxt=1;
                            if(s_tick==1) begin
                                if(s_reg==SB_TICK-1) begin
                                    tx_done_tick=1;
                                    state_nxt=idle;
                                end
                                else s_nxt=s_reg+1;
                            end
                        end
            default: state_nxt=idle;
         endcase
     end
     assign uart_tx=tx_reg;
    /*********************************************************************************/
    
    /******************************** UART RX ****************************************/
	 
	 //FSM register operation
	 always @(posedge clk,negedge rst_n) begin
		if(!rst_n) begin
			state_reg_rx<=idle;
			s_reg_rx<=0;
			n_reg_rx<=0;
			b_reg<=0;
			dout<=0;
			rx_buffer_full<=0;
		end
		else begin
			state_reg_rx<=state_nxt_rx;
			s_reg_rx<=s_nxt_rx;
			n_reg_rx<=n_nxt_rx;
			b_reg<=b_nxt;	
			if(rx_done_tick) begin
			    dout <= b_reg; //memory-mapped register storing the completed read data	
			    rx_buffer_full <= 1'b1; //memory-mapped register to check if a read is done
			end
			else if(i_stb_data && !uart_wr_en && uart_rw_address == UART_RX_DATA) begin //read request to UART_RX_DATA (read the data)
                rx_buffer_full <= 1'b0;
            end
		end
	 end
	 
	 //FSM next-state logic
	 always @* begin
		state_nxt_rx=state_reg_rx;
		s_nxt_rx=s_reg_rx;
		n_nxt_rx=n_reg_rx;
		b_nxt=b_reg;
		rx_done_tick=0;
		case(state_reg_rx)
			 idle: if(uart_rx==0) begin //wait for start bit(rx of zero)
						s_nxt_rx=0;
						state_nxt_rx=start;
					 end						
			start: if(s_tick==1) begin //wait for middle of start bit
						if(s_reg_rx==7) begin
							s_nxt_rx=0;
							n_nxt_rx=0;
							state_nxt_rx=data;
						end
						else s_nxt_rx=s_reg_rx+1;
					 end
		    data: if(s_tick==1) begin //wait to pass all middle points of every data bits
						if(s_reg_rx==15) begin
							b_nxt={uart_rx,b_reg[7:1]};
							s_nxt_rx=0;
							if(n_reg_rx==DBIT-1) state_nxt_rx=stop;
							else n_nxt_rx=n_reg_rx+1;
						end
						else s_nxt_rx=s_reg_rx+1;
					 end
			 stop: if(s_tick==1) begin  //wait to pass the required stop bits
						if(s_reg_rx==SB_TICK-1) begin
							rx_done_tick=1;
							state_nxt_rx=idle;
						end
  						else s_nxt_rx=s_reg_rx+1;
					 end	
		 default: state_nxt_rx=idle;
		endcase
	 end
	 /*********************************************************************************/
	 
endmodule