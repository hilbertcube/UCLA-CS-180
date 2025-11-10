#!/bin/bash

# Check for required argument
if [ -z "$1" ]; then
  echo "Usage: $0 <filename-without-extension>"
  exit 1
fi

p="$1"

# Ensure bin/ exists
mkdir -p bin

# Compile
g++ "$p.cpp" -o "bin/$p"
if [ $? -ne 0 ]; then
  echo "Compilation failed (g++ exited with code $?)"
  exit $?
fi

# Run
./bin/"$p"

# Make an executable: chmod +x run_cpp.sh
# Run: ./run_cpp.sh your_cpp_filename