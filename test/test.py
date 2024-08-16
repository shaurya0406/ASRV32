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

## Debug Variables ##
debug_objdump = False
debug_exception = False
debug_sbreak = False

## Function Definitions ##

##################################################### ENVIRONMENT SETUP #####################################################
def check_env(asrv32_path):
    # Check if the ASRV32 environment variable is set
    if asrv32_path is not None:
        print(f'The ASRV32 path is: {asrv32_path}')
    else:
        print('The ASRV32 environment variable is not set. Please set it to the top ASRV32 Directory')
        exit(1)  # Exit the program if the environment variable is not set

def check_riscv_tests(asrv32_path):
    risv_tests_repo_path = os.path.join(asrv32_path, 'test', 'riscv-tests')

    # Check if the repository exists
    if not os.path.exists(risv_tests_repo_path):
        print(f"riscv-tests repository not found in {risv_tests_repo_path}. Cloning...")
        
        # Clone the repository
        try:
            subprocess.run(['git', 'clone', '--recursive', 'https://github.com/riscv-software-src/riscv-tests.git', risv_tests_repo_path], check=True)
            print("Cloning successful.")
        except subprocess.CalledProcessError as e:
            print(f"Error during cloning: {e}")
            exit(1)
    else:
        print(f"riscv-tests repository already exists in {risv_tests_repo_path}.")

##################################################### ICARUS VERILOG COMPILE #####################################################
def compile_verilog(top_module, verilog_files, output_file):

    global debug_exception, debug_sbreak

    # Get the ASRV32 path from the environment variable
    asrv32_path = os.getenv('ASRV32')

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
    ]                

    if(debug_exception):
        compile_command.append('-DHALT_ON_ILLEGAL_INSTRUCTION')
    if(debug_sbreak):
        compile_command.append('-DHALT_ON_ECALL')
    compile_command+=verilog_files   # The 'verilog_files' list contains the paths to all the Verilog source files and the top testbench to be compiled; these are appended to the command list

    try:
        compile_process = subprocess.run(compile_command, capture_output=True, text=True, check=True)
        print("Icarus Compilation successful.")
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
        print("Icarus Simulation successful.")

        if(specific_testfile):
            # print("Simulation output:")
            # print(simulation_process.stdout)  # Print simulation output
            print(simulation_process.stderr)  # Print simulation errors

            if "PASS: exit code = 0x00000000" in simulation_process.stdout:
                countpassed += 1
                print(f"\nPASSED: {os.path.basename(specific_testfile)}\n")
            elif "FAIL" in simulation_process.stdout:
                countfailed += 1
                failedlist += f"{os.path.basename(specific_testfile)}\n"
            elif "UNKNOWN" in simulation_process.stdout:
                countunknown += 1
                filename = os.path.basename(os.path.basename(specific_testfile))
                unknownlist += f"{filename}\n"
            else:
                countunknown += 1
                unknownlist += f"{os.path.basename(specific_testfile)}\n"
        else:
            print("Simulation output:")
            print(simulation_process.stdout)  # Print simulation output
            print(simulation_process.stderr)  # Print simulation errors

    except subprocess.CalledProcessError as e:
        print("Simulation failed with the following errors:")
        print(e.stdout)
        print(e.stderr)
        exit(1)  # Exit if simulation fails

##################################################### GTKWAVE SIM #####################################################
def open_gtkwave(top_module, vcd_file):
    # Check if the VCD file exists
    if not os.path.exists(vcd_file):
        print(f"VCD file {vcd_file} not found. Ensure your testbench generates the VCD file.")
        exit(1)  # Exit if VCD file is not found

    # Define the command to open GTKWave with the VCD file
    gtkwave_command = ["gtkwave", f'{top_module}.gtkw']
    subprocess.run(gtkwave_command)  # Execute the command to open GTKWave

##################################################### RISCV GNU TOOLCHAIN #####################################################
# Function to compile a source file using RISC-V GCC
def compile_source_file(asrv32_path, source_file, obj_directory, march='rv32g', mabi='ilp32'):
    try:
        output_file = f'{obj_directory}/{os.path.basename(source_file).replace(".c", ".o").replace(".s", ".o")}'
        # Construct the GCC compile command
        compile_command = [
            'riscv64-unknown-elf-gcc',                                  # The RISC-V GCC compiler
            '-c',                                                       # Compile only; do not link
            '-g',                                                       # Include debugging information in the output
            f'-march={march}',                                          # Specify the target architecture
            f'-mabi={mabi}',                                            # Specify the target ABI (Application Binary Interface)
            '-I', f'{asrv32_path}/test/riscv-tests/env/p',              # Include directory for environment
            '-I', f'{asrv32_path}/test/riscv-tests/isa/macros/scalar',  # Include directory for ISA macros
            source_file,                                                # The source file to compile
            '-o', output_file                                           # Output the compiled object file
        ]
        subprocess.run(compile_command, check=True)
        return output_file
    except subprocess.CalledProcessError as e:
        print(f"Compilation failed for {source_file}:")
        print(e.stdout)
        print(e.stderr)
        return None

def link_object_file(obj_files, output_file):
    try:
        # Generate the binary file
        link_command = [
            'riscv64-unknown-elf-ld', '-melf32lriscv', '-Ttext', '0',
            obj_files, '-o', output_file
        ]
        subprocess.run(link_command, check=True)
        return output_file
    except subprocess.CalledProcessError as e:
        print(f"Linking failed:")
        print(e.stdout)
        print(e.stderr)
        return None

def generate_memory_file(elf_file, obj_directory):
    if(debug_objdump):
        print("\n##############################################################\n")
        try:
            # riscv64-unknown-elf-objdump -M numeric -D ${ONAME}.bin -h
            objdump_command =[
                'riscv64-unknown-elf-objdump', '-M', 'numeric', '-D', elf_file, '-h'
            ]
            subprocess.run(objdump_command, check=True)
        except subprocess.CalledProcessError as e:
            print(f"Obj Dump failed:")
            print(e.stdout)
            print(e.stderr)
        print( "\n##############################################################\n")
    
    try:
        memory_file = f'{obj_directory}/memory.mem'
        elf2hex_command = [
            'riscv64-unknown-elf-elf2hex', '--bit-width', '32',
            '--input', elf_file, '--output', memory_file
        ]
        subprocess.run(elf2hex_command, check=True)
        return memory_file
    except subprocess.CalledProcessError as e:
        print(f"Memory file generation failed:")
        print(e.stdout)
        print(e.stderr)
        return None
       
def run_tests(asrv32_path, top_module, test_type):
    global countfile, countpassed, countfailed, countunknown, countmissing, failedlist, unknownlist, missinglist
    global debug_objdump, debug_exception, debug_sbreak

    obj_directory = f"{asrv32_path}/test/obj"
    if os.path.exists(obj_directory):
        shutil.rmtree(obj_directory)
    os.makedirs(obj_directory)
    
    if test_type == "rv32ui":
        test_dir = f"{asrv32_path}/test/riscv-tests/isa/rv32ui/"
        test_files = glob.glob(f"{test_dir}*.S") + glob.glob(f"{test_dir}*.s") + glob.glob(f"{test_dir}*.c")
        debug_objdump = False
    elif test_type == "rv32mi":
        test_dir = f"{asrv32_path}/test/riscv-tests/isa/rv32mi/"
        test_files = glob.glob(f"{test_dir}*.S") + glob.glob(f"{test_dir}*.s") + glob.glob(f"{test_dir}*.c")
        debug_objdump = False
    elif test_type == "extra":
        test_dir = f"{asrv32_path}/test/testfiles/"
        test_files = glob.glob(f"{test_dir}*.S") + glob.glob(f"{test_dir}*.s") + glob.glob(f"{test_dir}*.c")
        debug_objdump = False
    elif test_type == "all":
        for t in ["rv32ui", "rv32mi", "extra"]:
            run_tests(asrv32_path, top_module, t)
        return
    else:
        debug_objdump = True
        test_files = [test_type]
        for test_file in test_files:
            if not os.path.exists(test_file):
                print(f"Test file {test_file} is missing.")
                countmissing += 1
                missinglist.append(test_file)
                return

    for test_file in test_files:
        countfile += 1
        dir_name = os.path.basename(os.path.dirname(test_file))
        test_name = os.path.basename(test_file)
        print(f"TEST:{countfile}\tTestfile: {dir_name}/{test_name}")

        if "exception" in test_file:
            debug_exception = True
            debug_sbreak = False
        elif "sbreak" in test_file:
            debug_exception = False
            debug_sbreak = True
        else:
            debug_exception = False
            debug_sbreak = False

        obj_file = compile_source_file(asrv32_path, test_file, obj_directory)
        if not obj_file:
            countmissing += 1
            missinglist.append(test_name)
            continue

        elf_file = f"{obj_directory}/{test_name.replace('.S', '.bin').replace('.s', '.bin').replace('.c', '.bin')}"
        link_object_file(obj_file, elf_file)
        generate_memory_file(elf_file, obj_directory)

        verilog_src_files = glob.glob(f"{asrv32_path}/rtl/*.v")
        verilog_tb_file = [f"{asrv32_path}/test/SoC_TB/{top_module}.v"]
        verilog_files = verilog_src_files + verilog_tb_file
        Icarus_out_file = f"{top_module}.vvp"
        compile_verilog(top_module, verilog_files, Icarus_out_file)

        simulate_verilog(top_module, Icarus_out_file, elf_file)

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
    parser = argparse.ArgumentParser(description='ASRV32 Compilation and Simulation Script')
    parser.add_argument('top_module', type=str, help='Top Verilog module name')
    parser.add_argument('action', choices=['compile', 'sim', 'sim_gui', 'test'], help='Action to perform')
    parser.add_argument('testfile', nargs='?', type=str, help='Test file path or test type (rv32ui, rv32mi, extra, all)', default=None)

    args = parser.parse_args()

    asrv32_path = os.getenv('ASRV32')
    check_env(asrv32_path)

    top_module = args.top_module
    action = args.action
    testfile = args.testfile

    if action == 'compile':
        verilog_src_files = glob.glob(f"{asrv32_path}/rtl/*.v")
        verilog_tb_file = [f"{asrv32_path}/test/SoC_TB/{top_module}.v"]
        verilog_files = verilog_src_files + verilog_tb_file
        Icarus_out_file = f"{asrv32_path}/{top_module}.vvp"
        compile_verilog(top_module, verilog_files, Icarus_out_file)
    
    elif action == 'sim':
        Icarus_out_file = f"{asrv32_path}/{top_module}.vvp"
        if not os.path.exists(Icarus_out_file):
            print(f"Compiled Verilog output {Icarus_out_file} not found. Please compile first.")
            exit(1)
        simulate_verilog(top_module, Icarus_out_file)

    elif action == 'sim_gui':
        Icarus_out_file = f"{asrv32_path}/{top_module}.vvp"
        vcd_file = f"{top_module}.vcd"
        if not os.path.exists(Icarus_out_file):
            print(f"Compiled Verilog output {Icarus_out_file} not found. Please compile first.")
            exit(1)
        simulate_verilog(top_module, Icarus_out_file)
        open_gtkwave(top_module, vcd_file)

    elif action == 'test':
        if not testfile:
            print("Test type or specific test file must be provided for the 'test' action.")
            exit(1)
        check_riscv_tests(asrv32_path)
        run_tests(asrv32_path, top_module, testfile)

if __name__ == "__main__":
    main()