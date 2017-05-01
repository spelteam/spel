REM @echo off

set configuration=%1
set platform=%2

call "%VS120COMNTOOLS%\vsvars32.bat"

REM devenv %solution_name% /Build "%configuration%|%platform%"
