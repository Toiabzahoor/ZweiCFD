@echo off
setlocal

:: Navigate to the directory containing CMakeLists.txt
cd /d "%~dp0"

echo Killing existing instances of ZweiCFD.exe and zweifoil.exe...
taskkill /F /IM ZweiCFD.exe /T >nul 2>&1
taskkill /F /IM zweifoil.exe /T >nul 2>&1

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
echo You can run the application with: .\build\ZweiCFD.exe
echo =========================================
exit /b 0
