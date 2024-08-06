import os           # Import the os module for environment variable access and path operations
import subprocess   # Import subprocess module to run external commands
import argparse     # Import argparse module for command-line argument parsing
import glob         # Import glob module for file pattern matching

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

def simulate_verilog(output_file):
    # Define the simulation command for the compiled Verilog file
    simulation_command = ["vvp", "-n", output_file]
    
    # Execute the simulation command
    try:
        simulation_process = subprocess.run(simulation_command, capture_output=True, text=True, check=True)
        print("Simulation successful.")
        print("Simulation output:")
        print(simulation_process.stdout)  # Print simulation output
        print(simulation_process.stderr)  # Print simulation errors
    except subprocess.CalledProcessError as e:
        print("Simulation failed with the following errors:")
        print(e.stdout)
        print(e.stderr)
        exit(1)  # Exit if simulation fails

def open_gtkwave(vcd_file):
    # Check if the VCD file exists
    if not os.path.exists(vcd_file):
        print(f"VCD file {vcd_file} not found. Ensure your testbench generates the VCD file.")
        exit(1)  # Exit if VCD file is not found

    # Define the command to open GTKWave with the VCD file
    gtkwave_command = ["gtkwave", vcd_file]
    subprocess.run(gtkwave_command)  # Execute the command to open GTKWave

def main():
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Automate Verilog Compilation and Simulation")
    parser.add_argument("top_module", help="Top module name")
    parser.add_argument("action", choices=["compile", "sim", "sim_gui", "all"], help="Action to perform")
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
        simulate_verilog(Icarus_out_file)
    
    if args.action in ["sim_gui", "all"]:
        open_gtkwave(vcd_file)

if __name__ == "__main__":
    main()  # Execute the main function if the script is run directly
