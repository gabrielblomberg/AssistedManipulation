#!/bin/bash

WORKSPACE=$(pwd)

# Add library directories to cmake path.
CMAKE_PREFIX_PATH+=";$WORKSPACE/lib/raisimlib/raisim/linux"
export CMAKE_PREFIX_PATH

# Add raisim linker directories to path.
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$WORKSPACE/raisim/linux/lib
export PYTHONPATH=$PYTHONPATH:$WORKSPACE/raisim/linux/lib

mkdir -p build

cmake \
    -DCMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH \
    -DCMAKE_FIND_USE_CMAKE_ENVIRONMENT_PATH=y \
    -B build \
    -S src
