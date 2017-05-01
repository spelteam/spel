REM @echo off

set configuration=%1
set platform=%2

call "%VS120COMNTOOLS%\vsvars32.bat"

cd ../
mkdir dependencies
cd dependencies

REM OpenGM
mkdir OpenGM
cd OpenGM
powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/opengm/opengm/archive/master.zip','C:\projects\dependencies\OpenGM\master.zip')
7z x master.zip
cd ../

REM OpenCV
mkdir opencv
cd opencv
powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/opencv/opencv/archive/3.2.0.zip','C:\projects\dependencies\opencv\3.2.0.zip')
7z x 3.2.0.zip
rm 3.2.0.zip
powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/opencv/opencv_contrib/archive/3.2.0.zip','C:\projects\dependencies\opencv\3.2.0.zip')
wget https://github.com/opencv/opencv_contrib/archive/3.2.0.zip
7z x 3.2.0.zip
rm 3.2.0.zip
mkdir build
cd build
cmake -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.2.0/modules -DBUILD_opencv_apps=OFF -DBUILD_DOCS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DBUILD_FAT_JAVA_LIB=OFF ../opencv-3.2.0
REM devenv OpenCV.sln /Build "%configuration%|%platform%"
dir
devenv C:\projects\dependencies\opencv\build\OpenCV.sln /Build "Debug|x64"
cd ../../

cd ../spel

mkdir build
cd build
cmake ../src/ -DOpenGM_INCLUDE_DIR='C:\projects\dependencies\OpenGM\opengm-master\include' -DOpenCV_DIR='C:\projects\dependencies\opencv\build'

REM devenv %solution_name% /Build "%configuration%|%platform%"
