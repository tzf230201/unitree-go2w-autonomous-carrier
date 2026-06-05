@echo off
setlocal
set "PATH=C:\msys64\mingw64\bin;%PATH%"
cd /d "%~dp0"
if not exist build (
    cmake -G Ninja -S . -B build -DCMAKE_BUILD_TYPE=Release || exit /b 1
)
cmake --build build || exit /b 1
echo.
echo Build OK. Run with: run.bat
