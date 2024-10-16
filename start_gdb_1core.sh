#!/bin/bash

# Open a new Kitty terminal window and run 'linkserver gdbserver lpc55s69'
kitty --hold bash -c "nohup /opt/linkserver/LinkServer gdbserver lpc55s69:lpcxpresso55s69 >/dev/tty 2>/dev/tty &" &


# Sleep for a few seconds to ensure the gdbserver starts properly before running GDB
sleep 3


arm-none-eabi-gdb main.elf -x .gdb_commands_1core.txt
