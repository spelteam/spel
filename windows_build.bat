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
curl -o 'master.zip' 'https://github.com/opengm/opengm/archive/master.zip'
7z x master.zip
REM powershell.exe -Command (new-object System.Net.WebClient).DownloadFile('https://github.com/opengm/opengm/archive/master.zip','C:\dependencies\OpenGM')
cd ../

cd ../spel

mkdir build
cd build
cmake ../src/

REM devenv %solution_name% /Build "%configuration%|%platform%"
