REM @echo off

call "%VS140COMNTOOLS%\vsvars32.bat"

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
7z x 3.2.0.zip
rm 3.2.0.zip
mkdir build
cd build
cmake -DCMAKE_GENERATOR_PLATFORM=x64 -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.2.0/modules -DBUILD_opencv_apps=OFF -DBUILD_DOCS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DBUILD_FAT_JAVA_LIB=OFF ../opencv-3.2.0
devenv OpenCV.sln /build "Debug|x64"
cd ../../

REM Eigen
mkdir eigen
cd eigen
powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('http://bitbucket.org/eigen/eigen/get/3.2.10.zip','C:\projects\dependencies\eigen\3.2.10.zip')
7z x unzip 3.2.10.zip
cmake ../eigen-eigen-b9cd8366d4e8
cd ../

REM Google Test
mkdir gtest
cd gtest
powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/google/googletest/archive/release-1.8.0.zip','C:\projects\dependencies\gtest\release-1.8.0.zip')
7z x release-1.8.0.zip
mv googletest-release-1.8.0/googletest/ ../../../thirdparty/
cd ../

REM TinyXml2
mkdir tinyxml
cd tinyxml
powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/leethomason/tinyxml2/archive/4.0.1.zip','C:\projects\dependencies\tinyxml\4.0.1.zip')
7z x 4.0.1.zip
mv tinyxml2-4.0.1/ ../../../thirdparty/tinyxml2
cd ../

cd ../spel

mkdir build
cd build
cmake ../src/ -DCMAKE_GENERATOR_PLATFORM=x64 -DOpenGM_INCLUDE_DIR='C:\projects\dependencies\OpenGM\opengm-master\include' -DOpenCV_DIR='C:\projects\dependencies\opencv\build' -DEigen3_INCLUDE_DIR='C:\projects\dependencies\eigen\eigen-eigen-b9cd8366d4e8'

REM devenv %solution_name% /Build "%configuration%|%platform%"
