# ASRV32 SoC Test Bench

This section provides an example of running the ASRV32 SoC test bench and the corresponding output.

## Setting up the Environment
Open the terminal at the Top Directory Location of ASRV32 Git Repo, once you have cloned it.

Command to run for cloning with HTTPS:
```sh
git clone --recursive https://github.com/shaurya0406/ASRV32.git
```
Set the ASRV32 Environment Variable using this command:
```sh
export ASRV32=`pwd`  
```
Check Python Version by running this command: (Code testsed with `Python 3.12.4`)
```sh
python3 --version 
```
## Running the Test Bench
To run the test bench, use the following command:
```sh
python3 ${ASRV32}/test/test.py asrv32_soc_TB test rv32ui
```

## Expected Output

```plaintext
The ASRV32 path is: /Users/shacha/ASRV32
riscv-tests repository not found in /Users/shacha/ASRV32/test/riscv-tests. Cloning...
Cloning into '/Users/shacha/ASRV32/test/riscv-tests'...
.
.
.
Cloning successful.
TEST:1  Testfile: rv32ui/fence_i.S
Icarus Compilation successful.


Icarus Simulation successful.


PASSED: fence_i.bin
.
.
.
TEST:40 Testfile: rv32ui/slti.S
Icarus Compilation successful.


Icarus Simulation successful.


PASSED: slti.bin


Test Summary
Total number of files: 40
Total number of PASSED: 40
Total number of FAILED: 0
Total number of UNKNOWN: 0
Total number of missing files: 0
```

## GTKWave Simulation Output

To run the simulation, use the following command:

```sh
python3 ${ASRV32}/test/test.py asrv32_soc_TB sim_gui
```