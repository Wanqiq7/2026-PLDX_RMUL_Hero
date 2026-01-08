#!/bin/bash

echo "--- Compiling project ---"
make -j 24
if [ $? -ne 0 ]; then
    echo "Error: Compilation failed. Please check Makefile and code."
    exit 1
fi

echo "--- Flashing firmware ---"
openocd -f openocd_dap.cfg -c "program build/basic_framework.elf verify reset exit"
if [ $? -ne 0 ]; then
    echo "Error: Flashing failed. Check hardware connection or OpenOCD config."
    exit 1
fi

echo "--- Done! ---"
exit 0