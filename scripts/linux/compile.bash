#!/bin/bash

set -e

if [ "$#" -ne 1 ]; then
    printf "usage: $0 (Release|Debug)\n"
    exit 1
fi

if [ "$1" -ne "Release" -a "$1" -ne "Debug" ]; then
    printf "usage: $0 (Release|Debug)\n"
    exit 1
fi

cd build
cmake --build . --config "%1"

cd ..
cmake --install build --config "%1" --prefix install
