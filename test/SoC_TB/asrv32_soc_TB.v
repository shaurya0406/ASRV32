`timescale 1ns / 1ps

module asrv32_soc_TB;

    /******************************* MODIFY ****************************************/
    localparam MEMORY_DEPTH = 2000,         //number of memory bytes
               DATA_START_ADDR = 32'h1080;  //starting address of data memory to be displayed
    /*******************************************************************************/
               
    reg clk,rst_n;
    integer i,j;          
    asrv32_soc #(.PC_RESET(32'h00_00_00_00), .MEMORY_DEPTH(MEMORY_DEPTH)) uut (
        .clk(clk),
        .rst_n(rst_n)
        );
    
    always #10 clk=!clk;
        
    /*********************** initialize instruction memory and data memory **************************/
    initial begin  
        #1; 
        $readmemh("test/SoC_TB/combined_inst_data.mem",uut.m1.memory_regfile); //write instruction and data to memory
    end
    /***********************************************************************************************/


    initial begin
        $dumpfile("asrv32_soc_TB.vcd"); // Same name as module name
        $dumpvars(0,asrv32_soc_TB);
        clk=0;
        rst_n=0;
        #100;
        
        rst_n=1; //release reset

        $display("\nStart executing instructions......\n");
        $display("Monitor All Writes to Base Register and Data Memory");
        
        while (uut.iaddr < MEMORY_DEPTH-4 && uut.m0.inst_q != 32'h00100073) begin //while instruction address is not yet at end of ROM or is not an ebreak instruction
            @(negedge clk);
            if(uut.m1.i_wr_en) begin //data memory is written
                $display("[MEMORY] address:0x%h   value:0x%h [MASK:%b]",uut.m1.i_data_addr,uut.m1.i_data_in,uut.m1.i_wr_mask); //display address of memory changed and its new value
            end
            if(uut.m0.m0.i_ce_wr && uut.m0.m0.i_rd_addr!=0) begin //base register is written
                $display("[BASEREG] address:0x%h   value:0x%h",uut.m0.m0.i_rd_addr,uut.m0.m0.i_rd_data); //display address of base reg changed and its new value
            end
        end
        $display("\nAll instructions executed......");
        
        /************* Dump Base Register and Memory Values *******************/
        $display("\nFinal Register State:");
        
        for(i=0; i<8; i=i+1) begin
            for(j=0; j<4 ; j=j+1) begin
                $write("0x%02d: 0x%h\t",4*i+j,uut.m0.m0.base_regfile[4*i+j]);
            end
            $write("\n");
        end
        $display("\n\nFinal Memory State:"); // Display first 10 data memory locations
        for(i=DATA_START_ADDR; i<(DATA_START_ADDR+10*4) ; i=i+4) begin
            $display("0x%0h: 0x%h",i,uut.m1.memory_regfile[i>>2]);
        end
        /**********************************************************/

        if(uut.m0.m0.base_regfile[17] == 32'h5d) begin //Exit test using RISC-V International's riscv-tests pass/fail criteria
                if(uut.m0.m0.base_regfile[10] == 0)
                    $display("\nPASS: exit code = 0x%h\n",uut.m0.m0.base_regfile[10]>>1);
                else begin
                    $display("\nFAIL: exit code = 0x%h\n",uut.m0.m0.base_regfile[10]>>1);
                end
            end
            else $display("\nUNKNOWN: basereg[17] = 0x%h (must be 0x0000005d)",uut.m0.m0.base_regfile[17]);
            
        $stop;
    end
endmodule
