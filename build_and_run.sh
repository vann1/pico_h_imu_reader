#!/bin/bash
# set -e

echo "Configuring cmake..."
cmake -S . -B build 
echo "Entering build directory..."
cd build

echo "Building project..."
make

echo "Build completed successufully"

cd ..
