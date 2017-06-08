#!/bin/bash

# configure the JS build
#source /home/ken/Desktop/emsdk_portable/emsdk_env.sh
mkdir -p build/js
cd build/js
~/Desktop/emscripten/emconfigure cmake -DPROFILE=ON -DEMBIND=ON -DCMAKE_BUILD_TYPE=Release -DCeres_DIR=/home/ken/Desktop/ceres-js/share/Ceres ../..
~/Desktop/emscripten/emmake make
#emmake make VERBOSE=1 

