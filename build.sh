#!/usr/bin/bash

# Build the project
echo "Building the ntrip client..."
mkdir -p build
cd build
cmake ..
make clean
make -j$(nproc)
cp -n ../grm.json ./
echo "Build complete."
