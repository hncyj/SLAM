#!/bin/bash

set -e

if [ -d "build" ]; then
    echo "Removing existing build directory..."
    rm -rf build
fi

mkdir build
cd build

cmake ..
cmake --build .

cd examples
./Draw