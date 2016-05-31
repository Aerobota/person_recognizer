#!/bin/bash
## build opencv #
cd opencv
mkdir build
cd build
cmake ..
make -j4

## build v4l2 #
cd ../../
./bootstrap.sh
./configure
make -j4
cd ../
