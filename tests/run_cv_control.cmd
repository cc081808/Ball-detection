@echo off
setlocal
cd /d "%~dp0.."
if defined VSCMD_ARG_TGT_ARCH goto compile
set "CV_VSWHERE=%ProgramFiles(x86)%\Microsoft Visual Studio\Installer\vswhere.exe"
if not exist "%CV_VSWHERE%" (
 echo Install the Visual Studio C++ build tools to run the controller simulations.
 exit /b 1
)
for /f "usebackq tokens=*" %%I in (`"%CV_VSWHERE%" -latest -products * -requires Microsoft.VisualStudio.Component.VC.Tools.x86.x64 -property installationPath`) do set "CV_VS_INSTALL=%%I"
if not defined CV_VS_INSTALL (
 echo No Visual Studio C++ toolchain found.
 exit /b 1
)
call "%CV_VS_INSTALL%\VC\Auxiliary\Build\vcvars64.bat" >nul
if errorlevel 1 exit /b 1
:compile
if not exist ".runtime" mkdir ".runtime"
cl /nologo /EHsc /std:c++14 /W4 /I"vex-cv\include" "tests\cv_control_tests.cpp" /Fe".runtime\cv_control_tests.exe" /Fo".runtime\cv_control_tests.obj"
if errorlevel 1 exit /b 1
".runtime\cv_control_tests.exe"
exit /b %errorlevel%
