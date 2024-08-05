# ASRV32
RISCV Processor in Verilog HDL. (Currently FSM based.)

## File Descriptions:

- **`asrv32_soc.v`**: System-on-Chip (SoC) module that integrates the `asrv32_core`, ROM (for instruction memory), and RAM (for data memory).
- **`asrv32_core.v`**: Top-level module for the RV32I core.
- **`asrv32_fsm.v`**: Finite State Machine (FSM) controller managing the fetch, decode, execute, memory access, and writeback processes.
- **`asrv32_basereg.v`**: Interface for the register file of the 32 integer base registers.
- **`asrv32_decoder.v`**: Module responsible for decoding 32-bit instructions (Decode Stage).
- **`asrv32_alu.v`**: Arithmetic Logic Unit (ALU) handling arithmetic and logic operations (Execute Stage).
- **`asrv32_memoryaccess.v`**: Controller managing data memory access (Memory Access Stage).
- **`asrv32_writeback.v`**: Controller determining the next Program Counter (`PC`) and destination register (`rd`) values (Writeback Stage).
- **`asrv32_soc_TB.v`**: Testbench for the `asrv32_soc` module.

## TODO

:white_check_mark: FSM based Core and SoC Integration
:black_square_button: Add more testcases for the testbench    
:black_square_button: Automate the testbench   
:black_square_button: Add CSR Module to support FreeRTOS  
:black_square_button: Convert FSM based core implementation to pipeline     
:black_square_button: Add RV32 Extensions