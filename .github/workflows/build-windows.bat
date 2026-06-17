@echo off
setlocal enabledelayedexpansion

:: Try to find Visual Studio
for /f "usebackq tokens=*" %%i in (`"C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe" -latest -property installationPath`) do (
  set "VS_PATH=%%i"
)

if not defined VS_PATH (
  echo Visual Studio not found
  exit /b 1
)

:: Call vcvarsall to set up the environment
call "!VS_PATH!\VC\Auxiliary\Build\vcvarsall.bat" x64

:: Run cmake or your build command
%*
