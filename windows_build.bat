REM @echo off

set configuration=%1
set platform=%2

call "%VS120COMNTOOLS%\vsvars32.bat"

mkdir build
cd build
cmake ../src/

REM devenv %solution_name% /Build "%configuration%|%platform%"
