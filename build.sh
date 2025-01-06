#!/usr/bin/bash

# Build the project
echo "Building the project..."
g++ main.cpp ntrip_client.cpp -o ntrip_client.o -lpthread
echo "Build complete."
