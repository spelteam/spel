#!/bin/sh

mkdir dependencies
cd dependencies
# OpenGM
mkdir opengm
cd opengm
wget https://github.com/opengm/opengm/archive/master.zip
unzip master.zip
mkdir build
cd build
cmake ../opengm-master && make && make install
cd ../../

cd ../
