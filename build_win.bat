:: This batch script is used to build the Maya plugin for multiple versions of Maya.

@echo off


:: ============================================
:: Configuration - Set these variables according to your environment.

:: Set the generator for CMake you want to use.
set "GENERATOR=Visual Studio 18 2026"

:: Set the root directory where the Maya Devkits are located.
set "DEVKIT_ROOT=D:\MayaDev"


:: ============================================
:: Main build loop for different Maya versions.

set "SOURCE_DIR=src"

call :Build 2027 "Autodesk_Maya_2027*_DEVKIT_Windows" v143 20
call :Build 2026 "Autodesk_Maya_2026*_DEVKIT_Windows" v143 17
call :Build 2025 "Autodesk_Maya_2025*_DEVKIT_Windows" v143 17
call :Build 2024 "Autodesk_Maya_2024*_DEVKIT_Windows" v143 17
call :Build 2023 "Autodesk_Maya_2023*_DEVKIT_Windows" v142
call :Build 2022 "Autodesk_Maya_2022*_DEVKIT_Windows" v142

exit


:: ============================================
:: Function: Build the plugin for a specific Maya version.

:Build
set "MAYA_VERSION=%~1"
set "DEVKIT_PATTERN=%~2"
set "MSVC=%~3"
set "CXX_STANDARD_VER=%~4"
set "DIRNAME=build/win64_%MAYA_VERSION%"

call :FindDevkit "%DEVKIT_PATTERN%"

if not defined DEVKIT_LOCATION (
    exit /b
)

echo.
echo ========================================
echo Maya %MAYA_VERSION%
echo DEVKIT_LOCATION=%DEVKIT_LOCATION%
echo BUILD_DIR=%DIRNAME%
echo TOOLSET=%MSVC%
echo CXX_STANDARD_VER=%CXX_STANDARD_VER%
echo ========================================

cmake -S "%SOURCE_DIR%" -B "%DIRNAME%" -G "%GENERATOR%" -A x64 -T "%MSVC%" -DCXX_STANDARD_VER=%CXX_STANDARD_VER%

cmake --build "%DIRNAME%"

exit /b


:: ============================================
:: Function: Find the Maya Devkit location based on the provided pattern.

:FindDevkit
set "DEVKIT_LOCATION="

for /d %%D in ("%DEVKIT_ROOT%\%~1") do (
    if exist "%%~fD\devkitBase\include\maya\MFnPlugin.h" (
        set "DEVKIT_LOCATION=%%~fD\devkitBase"
        goto :eof
    )
)

exit /b
