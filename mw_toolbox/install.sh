#!/bin/bash

script_dir=$(dirname "$0")
echo "The script is located in $script_dir"
cd $script_dir
cd third-party
cd MeshFix
rm -rf build
mkdir build 
cd build
cmake ..
make
echo "install mw_toolbox finish..."

