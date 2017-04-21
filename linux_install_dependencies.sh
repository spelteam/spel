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
cmake ../opengm-master && make && sudo make install
cd ../../

# OpenCV
mkdir opencv
cd opencv
wget https://github.com/opencv/opencv/archive/3.2.0.zip
unzip opencv-3.2.0.zip
wget https://github.com/opencv/opencv_contrib/archive/3.2.0.zip
unzip opencv_contrib-3.2.0.zip
mkdir build
cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.2.0/modules ../opencv-3.2.0 && make && sudo make install
cd ../../

cd ../
