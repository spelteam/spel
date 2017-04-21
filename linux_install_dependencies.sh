#!/bin/sh

mkdir dependencies
cd dependencies

# OpenGM
mkdir opengm
cd opengm
wget https://github.com/opengm/opengm/archive/master.zip
unzip master.zip > /dev/null 2>&1
mkdir build
cd build
cmake -DBUILD_EXAMPLES=OFF -DBUILD_TUTORIALS=OFF -dBUILD_TESTING=OFF ../opengm-master
make -j2
sudo make install
cd ../../

# OpenCV
mkdir opencv
cd opencv
wget https://github.com/opencv/opencv/archive/3.2.0.zip
unzip 3.2.0.zip > /dev/null 2>&1
rm 3.2.0.zip
wget https://github.com/opencv/opencv_contrib/archive/3.2.0.zip
unzip 3.2.0.zip > /dev/null 2>&1
rm 3.2.0.zip
mkdir build
cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.2.0/modules -DBUILD_opencv_apps=OFF -DBUILD_DOCS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DBUILD_FAT_JAVA_LIB=OFF ../opencv-3.2.0
make -j2
sudo make install
cd ../../

# Eigen
mkdir eigen
cd eigen
wget http://bitbucket.org/eigen/eigen/get/3.2.10.zip
unzip 3.2.10.zip > /dev/null 2>&1
mkdir build
cd build
cmake ../eigen-eigen-b9cd8366d4e8
sudo make install
cd ../../

cd ../
