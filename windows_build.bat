REM @echo off

call "%VS140COMNTOOLS%\vsvars32.bat"

REM cd ../
REM mkdir dependencies
REM cd dependencies

REM REM OpenGM
REM mkdir OpenGM
REM cd OpenGM
REM powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/opengm/opengm/archive/master.zip','C:\projects\dependencies\OpenGM\master.zip')
REM 7z x master.zip
REM cd ../

REM REM OpenCV
REM mkdir opencv
REM cd opencv
REM powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/opencv/opencv/archive/3.2.0.zip','C:\projects\dependencies\opencv\3.2.0.zip')
REM 7z x 3.2.0.zip
REM rm 3.2.0.zip
REM powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/opencv/opencv_contrib/archive/3.2.0.zip','C:\projects\dependencies\opencv\3.2.0.zip')
REM 7z x 3.2.0.zip
REM rm 3.2.0.zip
REM mkdir build
REM cd build
REM cmake -DCMAKE_GENERATOR_PLATFORM=x64 -DOPENCV_EXTRA_MODULES_PATH=../opencv_contrib-3.2.0/modules -DBUILD_opencv_apps=OFF -DBUILD_DOCS=OFF -DBUILD_PERF_TESTS=OFF -DBUILD_TESTS=OFF -DBUILD_FAT_JAVA_LIB=OFF ../opencv-3.2.0
REM devenv OpenCV.sln /build "Debug|x64"
REM cd ../../

REM REM Eigen
REM mkdir eigen
REM cd eigen
REM powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('http://bitbucket.org/eigen/eigen/get/3.2.10.zip','C:\projects\dependencies\eigen\3.2.10.zip')
REM 7z x unzip 3.2.10.zip
REM cmake ../eigen-eigen-b9cd8366d4e8
REM cd ../

REM REM HDF5
REM mkdir hdf5
REM cd hdf5
REM powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://support.hdfgroup.org/ftp/HDF5/releases/hdf5-1.8.14/bin/windows/extra/hdf5-1.8.14-win64-vs2013-shared.zip','C:\projects\dependencies\hdf5\hdf5-1.8.14-win64-vs2013-shared.zip')
REM 7z x hdf5-1.8.14-win64-vs2013-shared.zip
REM cd ../

REM REM Google Test
REM mkdir gtest
REM cd gtest
REM powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/google/googletest/archive/release-1.8.0.zip','C:\projects\dependencies\gtest\release-1.8.0.zip')
REM 7z x release-1.8.0.zip
REM mv googletest-release-1.8.0/googletest/ ../../../thirdparty/
REM cd ../

REM REM TinyXml2
REM mkdir tinyxml
REM cd tinyxml
REM powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/leethomason/tinyxml2/archive/4.0.1.zip','C:\projects\dependencies\tinyxml\4.0.1.zip')
REM 7z x 4.0.1.zip
REM mv tinyxml2-4.0.1/ ../../../thirdparty/tinyxml2
REM cd ../

REM cd ../spel

dir C:\Qt\5.8\msvc2015_64\lib\cmake\Qt5Core

REM mkdir build
REM cd build
REM cmake ../src/ -DCMAKE_GENERATOR_PLATFORM=x64 -DOpenGM_INCLUDE_DIR='C:\projects\dependencies\OpenGM\opengm-master\include' -DOpenCV_DIR='C:\projects\dependencies\opencv\build' -DEigen3_INCLUDE_DIR='C:\projects\dependencies\eigen\eigen-eigen-b9cd8366d4e8' -DHDF5_HL_IMPORT_LIB='C:\projects\dependencies\hdf5\HDF5-1.8.14-win64\lib\hdf5_hl.lib' -DHDF5_IMPORT_LIB='C:\projects\dependencies\hdf5\HDF5-1.8.14-win64\lib\hdf5.lib'
REM -DHDF5_INCLUDE_DIR='C:\projects\dependencies\hdf5\HDF5-1.8.14-win64\include' -DQt5Core_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5Core' -DQt5Gui_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5Gui' -DQt5Network_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5Network' -DQt5OpenGL_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5OpenGL' -DQt5Widgets_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5Widgets' -DQt5Xml_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5Xml' -DQt5XmlPatterns_DIR='C:\QT\5.8\MSVC2015_64\lib\cmake\Qt5XmlPatterns'