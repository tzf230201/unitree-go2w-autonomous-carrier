@echo off
setlocal
set "PATH=C:\msys64\mingw64\bin;%PATH%"
cd /d "%~dp0build"
DepthSensor_Buggy.exe
