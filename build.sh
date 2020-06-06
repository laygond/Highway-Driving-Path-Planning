#!/bin/bash
# Script to build all components from scratch, using the maximum available CPU power
#
# Given parameters are passed over to CMake.
# Examples:
#    * ./build.sh -DCMAKE_BUILD_TYPE=Debug
#    * ./build.sh VERBOSE=1
#
# Written by Bryan Laygond

cd `dirname $0` # where this bash script is contained.

if [ -d "build" ]; then
  rm -rf build
  echo Got rid of old build!
fi

echo Compiling new code
mkdir -p build
cd build
cmake ..
make $*
