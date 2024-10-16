#!/bin/bash
#
# Script to save usable versions of programs 
make clean
DATE=$(date +"%Y%m%d_%H%M%S")
mkdir .save/$DATE
cp -r .dep cfg CMSIS lib source startup .gdb_commands_1core.txt .gdb_commands_2core.txt setup.sh start_gdb_1core.sh start_gdb_2core.sh Makefile ./.save/$DATE
