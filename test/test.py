import os           # Import the os module for environment variable access and path operations
import subprocess   # Import subprocess module to run external commands
import argparse     # Import argparse module for command-line argument parsing
import glob         # Import glob module for file pattern matching
import shutil       # Import shutil to remove directories and their contents recursively

## GLobal Counters ##
countfile=0     # stores total number of testfiles
countpassed=0   # stores total number of PASSED
countfailed=0   # stores total number of FAILED
countunknown=0  # stores total number of UNKNOWN (basereg 0x17 is not 0x0000005d)
countmissing=0  # stores total number of missing testfiles
failedlist=""   # stores lists of testfiles that FAILED
unknownlist=""  # stores lists of testfiles that has UNKNOWN output
missinglist=""  # stores list of testfiles that are missing

## Function Definitions ##

def check_env(asrv32_path):
    # Check if the ASRV32 environment variable is set
    if asrv32_path is not None:
        print(f'The ASRV32 path is: {asrv32_path}')
    else:
        print('The ASRV32 environment variable is not set. Please set it to the top ASRV32 Directory')
        exit(1)  # Exit the program if the environment variable is not set

def compile_verilog(top_module, verilog_files, output_file):
    # Get the ASRV32 path from the environment variable
    asrv32_path = os.getenv('ASRV32')

    # sim_directory = f"{asrv32_path}/test/obj" # Check if the directory exists
    # if os.path.exists(sim_directory):
    #     shutil.rmtree(sim_directory)  # Removes directory and all its contents
    # os.makedirs(sim_directory) # Create the directory

    # Define the file paths
    Icarus_out_file = f"{asrv32_path}/{top_module}.vvp"  # Define the output file for the compiled Verilog

    # Check if Icarus_out_file exists
    if os.path.exists(Icarus_out_file):
        os.remove(Icarus_out_file)  # Remove the file

    # Define the compilation command for Icarus Verilog
    compile_command = [
        'iverilog',                   # The 'iverilog' command, which is the Icarus Verilog compiler
        '-o', output_file,            # The '-o' option specifies the output file name for the compiled result (in this case, a .vvp file)
        '-I', f'{asrv32_path}/rtl',   # The '-I' option specifies an include directory for additional Verilog source files; in this case, the 'rtl' directory within the ASRV32 path
        '-s', top_module,             # The '-s' option specifies the top module name to use as the simulation root
        '-DICARUS',                   # The '-DICARUS' option defines a preprocessor macro named 'ICARUS' for conditional compilation within the Verilog source files
    ] + verilog_files                 # The 'verilog_files' list contains the paths to all the Verilog source files and the top testbench to be compiled; these are appended to the command list


    # Execute the compilation command
    try:
        compile_process = subprocess.run(compile_command, capture_output=True, text=True, check=True)
        print("Compilation successful.")
        print(compile_process.stdout)  # Print compilation output
        print(compile_process.stderr)  # Print compilation errors
    except subprocess.CalledProcessError as e:
        print("Compilation failed with the following errors:")
        print(e.stdout)
        print(e.stderr)
        exit(1)  # Exit if compilation fails

def simulate_verilog(top_module, output_file, specific_testfile = None):
    global countpassed, countfailed, countunknown, failedlist, unknownlist

    vcd_file = f"{top_module}.vcd"  # Define the VCD file for simulation results
    # Check if vcd_file exists
    if os.path.exists(vcd_file):
        os.remove(vcd_file)  # Remove the file

    # Define the simulation command for the compiled Verilog file
    simulation_command = ["vvp", "-n", output_file]

    # Execute the simulation command
    try:
        simulation_process = subprocess.run(simulation_command, capture_output=True, text=True, check=True)
        print("Simulation successful.")

        if(specific_testfile):
            # print("Simulation output:")
            # print(simulation_process.stdout)  # Print simulation output
            print(simulation_process.stderr)  # Print simulation errors

            if "PASS: exit code = 0x00000000" in simulation_process.stdout:
                print(f"\nPASSED: {specific_testfile}\n")
                countpassed += 1
            elif "FAIL" in simulation_process.stdout:
                countfailed += 1
                failedlist += f"{specific_testfile}\n"
            elif "UNKNOWN" in simulation_process.stdout:
                countunknown += 1
                unknownlist += f"{specific_testfile}\n"
        else:
            print("Simulation output:")
            print(simulation_process.stdout)  # Print simulation output
            print(simulation_process.stderr)  # Print simulation errors

    except subprocess.CalledProcessError as e:
        print("Simulation failed with the following errors:")
        print(e.stdout)
        print(e.stderr)
        exit(1)  # Exit if simulation fails

def open_gtkwave(top_module, vcd_file):
    # Check if the VCD file exists
    if not os.path.exists(vcd_file):
        print(f"VCD file {vcd_file} not found. Ensure your testbench generates the VCD file.")
        exit(1)  # Exit if VCD file is not found

    # Define the command to open GTKWave with the VCD file
    gtkwave_command = ["gtkwave", f'{top_module}.gtkw']
    subprocess.run(gtkwave_command)  # Execute the command to open GTKWave

# Function to assemble the source file
def assemble_source_file(asrv32_path, testfile, obj_directory):
    asm_file = f"{asrv32_path}/test/testfiles/{testfile}.s"

    # Check if the assembly file exists
    if not os.path.exists(asm_file):
        print(f"Test file {asm_file} is missing.")
        return False

    try:
        # Assemble the file
        assemble_command = [
            'riscv64-unknown-elf-as', '-fpic', '-march=rv32i_zicsr',
            f'-aghlms={obj_directory}/memory.list', '-o', f'{obj_directory}/memory.o', asm_file
        ]
        subprocess.run(assemble_command, check=True)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error during assembly of {testfile}:")
        print(e.stdout)
        print(e.stderr)
        return False

# Function to link the object file and generate the binary
def link_object_file(asrv32_path, obj_directory):
    try:
        # Generate the binary file
        link_command = [
            'riscv64-unknown-elf-ld', '-melf32lriscv', '-Ttext', '0',
            f'{obj_directory}/memory.o', '-o', f'{obj_directory}/memory.bin'
        ]
        subprocess.run(link_command, check=True)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error during linking:")
        print(e.stdout)
        print(e.stderr)
        return False

# Function to generate the memory file using elf2hex
def generate_memory_file_with_elf2hex(asrv32_path, obj_directory, bit_width=32):
    try:
        # Define the input binary file and output memory file
        binary_file = f'{obj_directory}/memory.bin'
        memory_file = f'{obj_directory}/memory.mem'

        # Generate the memory file using the elf2hex command
        elf2hex_command = [
            'riscv64-unknown-elf-elf2hex',    # Command to use elf2hex
            '--bit-width', f'{bit_width}',    # Set the bit width (default is 32)
            '--input', binary_file,           # Specify the input binary file
            '--output', memory_file           # Specify the output memory file
        ]
        subprocess.run(elf2hex_command, check=True)
        return True
    except subprocess.CalledProcessError as e:
        print(f"Error during memory file generation with elf2hex:")
        print(e.stdout)
        print(e.stderr)
        return False
    
# Main function to call the above three functions
def assemble_and_generate_memory_files(asrv32_path, testfile):
    global countfile, countpassed, countfailed, countunknown, countmissing, failedlist, unknownlist, missinglist
    countfile += 1
    obj_directory = f"{asrv32_path}/test/obj"

    # Step 1: Assemble the source file
    if not assemble_source_file(asrv32_path, testfile, obj_directory):
        countmissing += 1
        missinglist += f"{testfile}\n"
        return False

    # Step 2: Link the object file to create the binary
    if not link_object_file(asrv32_path, obj_directory):
        countfailed += 1
        failedlist += f"{testfile}\n"
        return False

    # Step 3: Generate the memory file
    if not generate_memory_file_with_elf2hex(asrv32_path, obj_directory):
        countfailed += 1
        failedlist += f"{testfile}\n"
        return False

    print(f"Successfully processed {testfile}.")
    return True
       
def run_tests(asrv32_path, top_module, specific_testfile=None):
    global countfile, countpassed, countfailed, countunknown, countmissing, failedlist, unknownlist, missinglist
    
    if specific_testfile:
        testfiles = [specific_testfile]
    else:
        testfiles = glob.glob(f"{asrv32_path}/test/testfiles/*.s")
        testfiles = [os.path.basename(f).replace('.s', '') for f in testfiles]

    obj_directory = f"{asrv32_path}/test/obj" # Check if the directory exists
    if os.path.exists(obj_directory):
        shutil.rmtree(obj_directory)  # Removes directory and all its contents
        
    os.makedirs(obj_directory) # Create the directory

    for testfile in testfiles:
        print(f"TEST:{countfile}\tTestfile: {testfile}")
        if assemble_and_generate_memory_files(asrv32_path, testfile):
            verilog_src_files = glob.glob(f"{asrv32_path}/rtl/*.v")
            verilog_tb_file = [f"{asrv32_path}/test/SoC_TB/{top_module}.v"]
            verilog_files = verilog_src_files + verilog_tb_file
            Icarus_out_file = f"{top_module}.vvp"
            compile_verilog(top_module, verilog_files, Icarus_out_file)
            try:
                simulate_verilog(top_module, Icarus_out_file,testfile)
            except:
                print("!!! Simulation Stuck !!!")
        print("\n**********************************************************************************************\n")

    # Print the summary
    print("\nTest Summary")
    print(f"Total number of files: {countfile}")
    print(f"Total number of PASSED: {countpassed}")
    print(f"Total number of FAILED: {countfailed}")
    if failedlist:
        print("List of FAILED files:")
        print(failedlist)
    print(f"Total number of UNKNOWN: {countunknown}")
    if unknownlist:
        print("List of UNKNOWN files:")
        print(unknownlist)
    print(f"Total number of missing files: {countmissing}")
    if missinglist:
        print("List of missing files:")
        print(missinglist)

## Main Function ##
def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Automate Verilog Compilation and Simulation")
    parser.add_argument("top_module", help="Top module name")
    parser.add_argument("action", choices=["compile", "sim", "sim_gui", "test", "all"], help="Action to perform")
    parser.add_argument("filename", nargs='?', default=None, help="Optional: Specific test file to run (without extension)")
    args = parser.parse_args()

    # Get the ASRV32 path from the environment variable
    asrv32_path = os.getenv('ASRV32')
    check_env(asrv32_path)  # Check if the ASRV32 environment variable is set

    # Define input file names and paths
    verilog_src_files = glob.glob(f"{asrv32_path}/rtl/*.v")
    verilog_tb_file = [f"{asrv32_path}/test/SoC_TB/{args.top_module}.v"]
    verilog_files = verilog_src_files + verilog_tb_file

    Icarus_out_file = f"{args.top_module}.vvp"  # Define the output file for the compiled Verilog
    vcd_file = f"{args.top_module}.vcd"  # Define the VCD file for simulation results

    # Perform actions based on the command-line argument
    if args.action in ["compile", "all"]:
        compile_verilog(args.top_module, verilog_files, Icarus_out_file)
    
    if args.action in ["sim", "sim_gui", "all"]:
        simulate_verilog(args.top_module, Icarus_out_file)
    
    if args.action in ["sim_gui", "all"]:
        open_gtkwave(args.top_module, vcd_file)

    if args.action in ["test"]:
        run_tests(asrv32_path, args.top_module, args.filename)

if __name__ == "__main__":
    main()  # Execute the main function if the script is run directly
