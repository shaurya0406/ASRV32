`timescale 1ns/1ps
`default_nettype none

`include "asrv32_header.vh"

/* Decoder Module & Ports Declaration*/
module asrv32_decoder
    (
        /* Inputs */
        input wire i_clk,           // Added clock and reset to register the outputs
        input wire i_rst_n,             
        input wire[31:0] i_inst,    // The 32-bit instruction to be decoded.
        
        /* Outputs */
        // o_rs1_addr and o_rs2_addr are already registered in the basereg module, hence wire datatype here in decoder definition
        output wire[4:0] o_rs1_addr,    // Address for register source 1. 
        output wire[4:0] o_rs2_addr,    // Address for register source 2.
        // Leaving o_rd_addr as is for now..
        output wire[4:0] o_rd_addr,     // Address for the destination register.
        // Need to hold value for later decoding, hence changed from wire to reg type
        output reg[31:0] o_imm,         // The sign-extended immediate value extracted from the instruction.
        output reg[2:0] o_funct3,       // Function type (3-bit field from the instruction). 
        output reg[`OPCODE_WIDTH-1:0] o_opcode, // Opcode type of the instruction.
        output reg[`ALU_WIDTH-1:0] o_alu_op,    // ALU operation type.
        output reg[`EXCEPTION_WIDTH-1:0] o_exception // Exceptions: illegal inst, ecall, ebreak, mret

    );

/* Internal Signals and Registers to hold input values for decoding */

    /* Opcode Types */
    reg opcode_rtype_d;
    reg opcode_itype_d;
    reg opcode_load_d;
    reg opcode_store_d;
    reg opcode_branch_d;
    reg opcode_jal_d;
    reg opcode_jalr_d;
    reg opcode_lui_d;
    reg opcode_auipc_d;
    reg opcode_system_d;
    reg opcode_fence_d;
    /* ALU Operations */
    reg alu_add_d;
    reg alu_sub_d;
    reg alu_slt_d;
    reg alu_sltu_d;
    reg alu_xor_d;
    reg alu_or_d;
    reg alu_and_d;
    reg alu_sll_d;
    reg alu_srl_d;
    reg alu_sra_d;
    reg alu_eq_d; 
    reg alu_neq_d;
    reg alu_ge_d; 
    reg alu_geu_d;
    /* Exceptions */
    reg system_noncsr = 0;
    reg valid_opcode = 0;
    reg illegal_shift = 0;

/* Functionality Outline: */

/* Operand Address, Opcode & Function Type Extraction: 
- The addresses for the source registers (rs1, rs2) and the destination register (rd) are extracted from the instruction.
- These addresses are used to read the values from the register file in the next stage.
*/

    //o_rs1_addr and o_rs2_addr are not registered since asrv32_basereg module do the registering itself
    assign o_rs1_addr = i_inst[19:15]; 
    assign o_rs2_addr = i_inst[24:20];  
    assign o_rd_addr = i_inst[11:7];

    wire[2:0] funct3_d = i_inst[14:12];
    wire[6:0] opcode = i_inst[6:0];

    reg[31:0] imm_d;

/* ALU Operation Decoding:
- Identify the type of instruction based on its opcode.
- Based on the opcode and function fields (funct3), the required ALU operation is decoded.
- Control signals are set to indicate the desired ALU operation to be performed in the execution stage.
*/

    // Assign Opcodes
    // always @(*) begin
    //     opcode_rtype_d  = opcode == `OPCODE_RTYPE;
    //     opcode_itype_d  = opcode == `OPCODE_ITYPE;
    //     opcode_load_d   = opcode == `OPCODE_LOAD;
    //     opcode_store_d  = opcode == `OPCODE_STORE;
    //     opcode_branch_d = opcode == `OPCODE_BRANCH;
    //     opcode_jal_d    = opcode == `OPCODE_JAL;
    //     opcode_jalr_d   = opcode == `OPCODE_JALR;
    //     opcode_lui_d    = opcode == `OPCODE_LUI;
    //     opcode_auipc_d  = opcode == `OPCODE_AUIPC;
    //     opcode_system_d = opcode == `OPCODE_SYSTEM;
    //     opcode_fence_d  = opcode == `OPCODE_FENCE;
    // end

    // Decode operation for ALU
    always @* begin
        // imm_d = 0;
        alu_add_d = 0;
        alu_sub_d = 0;
        alu_slt_d = 0;
        alu_sltu_d = 0;
        alu_xor_d = 0;
        alu_or_d = 0;
        alu_and_d = 0;
        alu_sll_d = 0;
        alu_srl_d = 0;
        alu_sra_d = 0;
        alu_eq_d = 0; 
        alu_neq_d = 0;
        alu_ge_d = 0; 
        alu_geu_d = 0;
        
        /********** Decode ALU Operation **************/
        if(opcode == `OPCODE_RTYPE || opcode == `OPCODE_ITYPE) begin
            if(opcode == `OPCODE_RTYPE) begin
                alu_add_d = funct3_d == `FUNCT3_ADD ? !i_inst[30] : 0; // add and sub has same o_funct3 code
                alu_sub_d = funct3_d == `FUNCT3_ADD ? i_inst[30] : 0;  // differs on i_inst[30]
            end
            else alu_add_d = funct3_d == `FUNCT3_ADD;

            alu_slt_d   = funct3_d == `FUNCT3_SLT;
            alu_sltu_d  = funct3_d == `FUNCT3_SLTU;
            alu_xor_d   = funct3_d == `FUNCT3_XOR;
            alu_or_d    = funct3_d == `FUNCT3_OR;
            alu_and_d   = funct3_d == `FUNCT3_AND;
            alu_sll_d   = funct3_d == `FUNCT3_SLL;
            alu_srl_d   = funct3_d == `FUNCT3_SRA ? !i_inst[30]:0;  // srl and sra has same o_funct3 code differs on i_inst[30]
            alu_sra_d   = funct3_d == `FUNCT3_SRA ? i_inst[30]:0; 
        end

        else if(opcode == `OPCODE_BRANCH) begin
           alu_eq_d     = funct3_d == `FUNCT3_EQ;
           alu_neq_d    = funct3_d == `FUNCT3_NEQ;    
           alu_slt_d    = funct3_d == `FUNCT3_LT;
           alu_ge_d     = funct3_d == `FUNCT3_GE;
           alu_sltu_d   = funct3_d == `FUNCT3_LTU;
           alu_geu_d    = funct3_d == `FUNCT3_GEU;
        end

        else alu_add_d = 1'b1; //add operation for all remaining instructions 
    end

/* Immediate Value Extraction:
- Depending on the instruction type, the immediate value is extracted and sign-extended. This immediate value can be used in arithmetic operations, load/store instructions, or as an offset for branches and jumps.
*/

    always @(*) begin
        case(opcode)
        `OPCODE_ITYPE , `OPCODE_LOAD , `OPCODE_JALR: imm_d = {{20{i_inst[31]}},i_inst[31:20]}; 
                                      `OPCODE_STORE: imm_d = {{20{i_inst[31]}},i_inst[31:25],i_inst[11:7]};
                                     `OPCODE_BRANCH: imm_d = {{19{i_inst[31]}},i_inst[31],i_inst[7],i_inst[30:25],i_inst[11:8],1'b0};
                                        `OPCODE_JAL: imm_d = {{11{i_inst[31]}},i_inst[31],i_inst[19:12],i_inst[20],i_inst[30:21],1'b0};
                        `OPCODE_LUI , `OPCODE_AUIPC: imm_d = {i_inst[31:12],12'h000};
                     `OPCODE_SYSTEM , `OPCODE_FENCE: imm_d = {20'b0,i_inst[31:20]};   
                                            default: imm_d = 0;
        endcase
    end

/* Assigning Decoded Outputs:
- Register the outputs of this decoder module for shorter combinational timing paths
- These decoded signals control the flow of data and determine the operations required in the next stages.
*/

    always @(posedge i_clk, negedge i_rst_n) begin
            if(!i_rst_n) begin
                o_funct3   <= 0;
                o_imm      <= 0; 
                /// Opcode Type ///
                o_opcode[`RTYPE]  <= 0;
                o_opcode[`ITYPE]  <= 0;
                o_opcode[`LOAD]   <= 0;
                o_opcode[`STORE]  <= 0;
                o_opcode[`BRANCH] <= 0;
                o_opcode[`JAL]    <= 0;
                o_opcode[`JALR]   <= 0;
                o_opcode[`LUI]    <= 0;
                o_opcode[`AUIPC]  <= 0;
                o_opcode[`SYSTEM] <= 0;
                o_opcode[`FENCE]  <= 0;
                /// ALU Operation ///
                o_alu_op[`ADD]  <= 0;
                o_alu_op[`SUB]  <= 0;
                o_alu_op[`SLT]  <= 0;
                o_alu_op[`SLTU] <= 0;
                o_alu_op[`XOR]  <= 0;
                o_alu_op[`OR]   <= 0;
                o_alu_op[`AND]  <= 0;
                o_alu_op[`SLL]  <= 0;
                o_alu_op[`SRL]  <= 0;
                o_alu_op[`SRA]  <= 0;
                o_alu_op[`EQ]   <= 0;
                o_alu_op[`NEQ]  <= 0;
                o_alu_op[`GE]   <= 0;
                o_alu_op[`GEU]  <= 0;
                /// Exceptions ///
                o_exception[`ILLEGAL]   <= 0;
                o_exception[`ECALL]     <= 0;
                o_exception[`EBREAK]    <= 0;
                o_exception[`MRET]      <= 0;
                
            end
            else begin
                o_funct3    <= funct3_d;
                o_imm       <= imm_d;

                opcode_rtype_d  = opcode == `OPCODE_RTYPE;
                opcode_itype_d  = opcode == `OPCODE_ITYPE;
                opcode_load_d   = opcode == `OPCODE_LOAD;
                opcode_store_d  = opcode == `OPCODE_STORE;
                opcode_branch_d = opcode == `OPCODE_BRANCH;
                opcode_jal_d    = opcode == `OPCODE_JAL;
                opcode_jalr_d   = opcode == `OPCODE_JALR;
                opcode_lui_d    = opcode == `OPCODE_LUI;
                opcode_auipc_d  = opcode == `OPCODE_AUIPC;
                opcode_system_d = opcode == `OPCODE_SYSTEM;
                opcode_fence_d  = opcode == `OPCODE_FENCE;

                //// Opcode Type ////
                o_opcode[`RTYPE]  <= opcode_rtype_d;
                o_opcode[`ITYPE]  <= opcode_itype_d;
                o_opcode[`LOAD]   <= opcode_load_d;
                o_opcode[`STORE]  <= opcode_store_d;
                o_opcode[`BRANCH] <= opcode_branch_d;
                o_opcode[`JAL]    <= opcode_jal_d;
                o_opcode[`JALR]   <= opcode_jalr_d;
                o_opcode[`LUI]    <= opcode_lui_d;
                o_opcode[`AUIPC]  <= opcode_auipc_d;
                o_opcode[`SYSTEM] <= opcode_system_d;
                o_opcode[`FENCE]  <= opcode_fence_d;
                /// ALU Operations ////
                o_alu_op[`ADD]  <= alu_add_d;
                o_alu_op[`SUB]  <= alu_sub_d;
                o_alu_op[`SLT]  <= alu_slt_d;
                o_alu_op[`SLTU] <= alu_sltu_d;
                o_alu_op[`XOR]  <= alu_xor_d;
                o_alu_op[`OR]   <= alu_or_d; 
                o_alu_op[`AND]  <= alu_and_d;
                o_alu_op[`SLL]  <= alu_sll_d; 
                o_alu_op[`SRL]  <= alu_srl_d;
                o_alu_op[`SRA]  <= alu_sra_d;
                o_alu_op[`EQ]   <= alu_eq_d; 
                o_alu_op[`NEQ]  <= alu_neq_d;
                o_alu_op[`GE]   <= alu_ge_d; 
                o_alu_op[`GEU]  <= alu_geu_d;

                /*********************** Decode possible exceptions ***********************/
                system_noncsr = opcode == `OPCODE_SYSTEM && funct3_d == 0 ; //system instruction but not CSR operation

                // Check if instruction is illegal    
                valid_opcode = (opcode_rtype_d || opcode_itype_d || opcode_load_d || opcode_store_d || opcode_branch_d || opcode_jal_d || opcode_jalr_d || opcode_lui_d || opcode_auipc_d || opcode_system_d || opcode_fence_d);
                illegal_shift = (opcode_itype_d && (alu_sll_d || alu_srl_d || alu_sra_d)) && i_inst[25];

                o_exception[`ILLEGAL] <= !valid_opcode || illegal_shift;

                // Check if ECALL
                o_exception[`ECALL] <= (system_noncsr && i_inst[21:20]==2'b00)? 1:0;
                
                // Check if EBREAK
                o_exception[`EBREAK] <= (system_noncsr && i_inst[21:20]==2'b01)? 1:0;
                
                // Check if MRET
                o_exception[`MRET] <= (system_noncsr && i_inst[21:20]==2'b10)? 1:0;
                /***************************************************************************/

            end
        end
endmodule