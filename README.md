# UPOLS_Algorithm
# A Uniformly Partitioned Overlap-Save Algorithm

# This project is an implementation of the UPOLS Algorithm in a Jupyter Notebook and on the LPCXpresso55S69 board.

# Project files:
# - jupyter/   : Contains Jupyter Notebook implementations
# - source/    : Source files for LPCXpresso board
# - lib/       : Libraries required for the project
# - cfg/       : Configuration files
# - CMSIS/     : CMSIS files for ARM Cortex-M processors

# Available scripts:
# - setup.sh            : Setup environment for compiling
# - start_gdb_1core.sh  : Starts GDB debugger for single-core execution
# - start_gdb_2core.sh  : Starts GDB debugger for dual-core execution

# Debugging procedure:
# 1. Run `. ./setup.sh` to set up the environment before running `make`.
#    Note: MCUXpresso IDE must be installed in the `/opt` directory.
#    (You may need to modify `setup.sh` if the compiler tools are not found.)
# 2. Compile the project by running `make`.
# 3. Start GDB with the script `./start_gdb_1core.sh` for debugging on one core.
