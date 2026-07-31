@echo off
setlocal

:: Navigate to the directory containing CMakeLists.txt
cd /d "%~dp0ZweiCFD"

echo =========================================
echo Configuring project with CMake (Ninja)...
echo =========================================
cmake -G Ninja -B build
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] CMake configuration failed!
    exit /b %errorlevel%
)

echo.
echo =========================================
echo Building project...
echo =========================================
cmake --build build
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Build failed!
    exit /b %errorlevel%
)

echo.
echo =========================================
echo Build successful! 
echo You can run the application with: .\ZweiCFD\build\ZweiFoil.exe
echo =========================================
exit /b 0
