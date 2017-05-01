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

cd ../spel

mkdir build
cd build
cmake ../src/

REM devenv %solution_name% /Build "%configuration%|%platform%"
