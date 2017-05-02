@echo off

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
7z x 3.2.10.zip
cd ../

REM HDF5
mkdir hdf5
cd hdf5
powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://support.hdfgroup.org/ftp/HDF5/releases/hdf5-1.8.14/bin/windows/extra/hdf5-1.8.14-win64-vs2013-shared.zip','C:\projects\dependencies\hdf5\hdf5-1.8.14-win64-vs2013-shared.zip')
7z x hdf5-1.8.14-win64-vs2013-shared.zip
cd hdf5-1.8.14
7z x HDF5-1.8.14-win64.exe
cd ../../

REM Google Test
mkdir gtest
cd gtest
powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/google/googletest/archive/release-1.8.0.zip','C:\projects\dependencies\gtest\release-1.8.0.zip')
7z x release-1.8.0.zip
mv googletest-release-1.8.0/googletest/ ../../spel/thirdparty/
cd ../

REM TinyXml2
mkdir tinyxml
cd tinyxml
powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/leethomason/tinyxml2/archive/4.0.1.zip','C:\projects\dependencies\tinyxml\4.0.1.zip')
7z x 4.0.1.zip
mv tinyxml2-4.0.1/ ../../spel/thirdparty/tinyxml2
cd ../

cd ../spel

mkdir build
cd build
cmake ../src/ -DCMAKE_GENERATOR_PLATFORM=x64 -DOpenGM_INCLUDE_DIR='C:\projects\dependencies\OpenGM\opengm-master\include' -DOpenCV_DIR='C:\projects\dependencies\opencv\build' -DEigen3_INCLUDE_DIR='C:\projects\dependencies\eigen\eigen-eigen-b9cd8366d4e8' -DHDF5_HL_IMPORT_LIB='C:\projects\dependencies\hdf5\hdf5-1.8.14\lib\hdf5_hl.lib' -DHDF5_IMPORT_LIB='C:\projects\dependencies\hdf5\hdf5-1.8.14\lib\hdf5.lib' -DHDF5_INCLUDE_DIR='C:\projects\dependencies\hdf5\hdf5-1.8.14\include' -DQt5Core_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5Core' -DQt5Gui_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5Gui' -DQt5Network_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5Network' -DQt5OpenGL_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5OpenGL' -DQt5Widgets_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5Widgets' -DQt5Xml_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5Xml' -DQt5XmlPatterns_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5XmlPatterns'
devenv spel.sln /build "Debug|x64"