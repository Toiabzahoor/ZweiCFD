@echo off
setlocal

:: Navigate to the directory containing CMakeLists.txt
cd /d "%~dp0"

echo Killing existing instances of ZweiCFD.exe...
taskkill /F /IM ZweiCFD.exe /T >nul 2>&1

echo =========================================
echo Configuring project with CMake (Ninja)...
echo =========================================
cmake -G Ninja -B build -DCMAKE_POLICY_VERSION_MINIMUM=3.5 -DCMAKE_BUILD_TYPE=Release
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] CMake configuration failed!
    exit /b %errorlevel%
)

echo.
echo =========================================
echo Building project in Release mode...
echo =========================================
cmake --build build --config Release
if %errorlevel% neq 0 (
    echo.
    echo [ERROR] Build failed!
    exit /b %errorlevel%
)

echo.
echo =========================================
echo Build successful! 
echo Launching ZweiCFD...
echo =========================================
start .\build\ZweiCFD.exe
exit /b 0
