`timescale 1ns / 1ps
`default_nettype none
`define DISPLAY
`include "asrv32_header.vh"

/* Declare Testbench Module and its Signals */
module asrv32_decoder_TB;

    reg clk;
    reg rst_n;
	reg[31:0] inst;         //32 bit instruction
	wire[4:0] rs1_addr;     //address for register source 1
	wire[4:0] rs2_addr;     //address for register source 2
	wire[4:0] rd_addr;      //address for destination address
	wire signed[31:0] imm;  //extended value for immediate
	wire[`ALU_WIDTH-1:0] decoder_alu;
    wire[`OPCODE_WIDTH-1:0] decoder_opcode;
	wire[2:0] funct3;

	reg[35:0] inst_regfile[15:0];               // Array to store test instructions
	reg[6*8-1:0] op_string,inst_type_string;    // Strings to store the decoded operation and instruction type
    integer i;                                  // Loop variable

    /* Instantiate the DUT (Device Under Test) */
	asrv32_decoder uut(
        .i_clk(clk),
        .i_rst_n(rst_n),
		.i_inst(inst),
		.o_rs1_addr(rs1_addr),
		.o_rs2_addr(rs2_addr),
		.o_rd_addr(rd_addr),
		.o_imm(imm),
        .o_funct3(funct3),
        .o_opcode(decoder_opcode),
		.o_alu_op(decoder_alu)	
	);

    /* Clock generation : 50 Mhz */
    always #10 clk=!clk;
    
    /* Assign relevant strings to Decoder Output for ease of debugging */

    //string value of ALU op (operation)
    always @(posedge clk)begin

        // String value for instruction type based on Opcode 
        case(1'b1)
            decoder_opcode[`RTYPE] : inst_type_string="R_TYPE";
            decoder_opcode[`ITYPE] : inst_type_string="I_TYPE";
            decoder_opcode[`LOAD]  : inst_type_string="LOAD";
            decoder_opcode[`STORE] : inst_type_string="STORE";
            decoder_opcode[`BRANCH]: inst_type_string="BRANCH";
            decoder_opcode[`JAL]   : inst_type_string="JAL";
            decoder_opcode[`JALR]  : inst_type_string="JALR";
            decoder_opcode[`LUI]   : inst_type_string="LUI";
            decoder_opcode[`AUIPC] : inst_type_string="AUIPC";
            decoder_opcode[`SYSTEM]: inst_type_string="SYSTEM";
            decoder_opcode[`FENCE] : inst_type_string="FENCE";
            default: inst_type_string="XXXX";
        endcase

        // String value of ALU operation
        case (1'b1)
            decoder_alu[`ADD]:  op_string = "ADD";
            decoder_alu[`SUB]:  op_string = "SUB";
            decoder_alu[`SLT]:  op_string = "SLT";
            decoder_alu[`SLTU]: op_string = "SLTU";
            decoder_alu[`XOR]:  op_string = "XOR";
            decoder_alu[`OR]:   op_string = "OR";
            decoder_alu[`AND]:  op_string = "AND";
            decoder_alu[`SLL]:  op_string = "SLL";
            decoder_alu[`SRL]:  op_string = "SRL";
            decoder_alu[`SRA]:  op_string = "SRA";
            decoder_alu[`EQ]:   op_string = "EQ";
            decoder_alu[`NEQ]:  op_string = "NEQ";
            decoder_alu[`GE]:   op_string = "GE";
            decoder_alu[`GEU]:  op_string = "GEU";
            default:            op_string = "XXXX";
        endcase
        
    end
	    
    /* Initialize test instructions */
    initial begin 
        inst_regfile[0]=32'b0100000_10000_01000_000_11111_0110011;  //test 1:   sub x31,x8,x16 (R-type)
        inst_regfile[1]=32'b1111111_11101_00010_100_00001_0010011;  //test 2:   xori x1,x2,-3 (I-type arithmetic)
        inst_regfile[2]=32'b1111111_11110_01000_000_00011_0000011;  //test 3:   lb x3,-2(x8) (Load)
        inst_regfile[3]=32'b0000000_10001_00111_001_00001_0100011;  //test 4:   sh x17,1(x7) (Store)
        inst_regfile[4]=32'b0000000_01110_11000_111_00010_1100011;  //test 5:   bgeu x24,x14,2 (Branch)
        inst_regfile[5]=32'b1111111_11111_11111_111_00001_1101111;  //test 6:   jal x1,-2 (JAL)
        inst_regfile[6]=32'b0000000_00000_00000_000_00001_1100111;  //test 7:   jalr x1,0(x1) (JALR)
        inst_regfile[7]=32'b0100000_00000_00000_001_10000_0110111;  //test 8:   lui 16,1_073_745_920
        inst_regfile[8]=32'b0100000_00000_00000_001_10000_0010111;  //test 9:   auipc 16,1_073_745_920
        inst_regfile[9]=32'b0000000_00000_00000_000_00000_1110011;  //test 10:  ecall
        inst_regfile[10]=32'b0000000_00000_00000_000_00000_0001111; //test 11:  fence
    end
    
    /* Test sequence */
	initial begin
        $dumpfile("decoder_wave.vcd");
        $dumpvars(0,asrv32_decoder_TB);
        clk = 0;    // Initialize clock as disabled
        rst_n = 0;  // Assert Reset
        #50
        rst_n = 1; // Disable Reset and Start Execution
        $display("\nStart Decoding Instructions......\n");

		for(i=0;i<=10;i=i+1) begin //iterate through all instructions
		    inst=inst_regfile[i][31:0];
            #100; // Wait for the instruction to be decoded
            $display("TEST: %d",i);
            $display("INST: %b_%b_%b_%b_%b_%b",inst_regfile[i][31:25],inst_regfile[i][24:20],inst_regfile[i][19:15],inst_regfile[i][14:12],inst_regfile[i][11:7],inst_regfile[i][6:0]);
            $display("inst_type=%s\nop=%s\nrs1_addr=%b\nrs2_addr=%b\nrd_addr=%b\nimm=%b_%b_%b_%b=%0d\n",inst_type_string,op_string,rs1_addr,rs2_addr,rd_addr,imm[31:24],imm[23:16],imm[15:8],imm[7:0],imm);
            #100; // Wait before the next iteration
        end
	    $display("\nAll Instructions Decoded......");
	    $stop;
	end

endmodule