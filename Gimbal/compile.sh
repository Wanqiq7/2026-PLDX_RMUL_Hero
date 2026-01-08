echo "--- Compiling project ---"
make -j 24
if [ $? -ne 0 ]; then
    echo "Error: Compilation failed. Please check Makefile and code."
    exit 1
fi

echo "--- Done! ---"
exit 0