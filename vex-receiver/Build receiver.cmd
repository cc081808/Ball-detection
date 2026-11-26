@echo off
setlocal
cd /d "%~dp0"
set "VEX_BASE=%APPDATA%\Code\User\globalStorage\vexrobotics.vexcode"
set "VEX_TOOLS=%VEX_BASE%\tools\cpp\toolchain_win32"
set "PATH=%VEX_TOOLS%\clang\bin;%VEX_TOOLS%\gcc\bin;%VEX_TOOLS%\tools\bin;%PATH%"
if not exist "%VEX_TOOLS%\tools\bin\make.exe" (
 echo Use Build in the VS Code VEX extension to install or locate its toolchain.
 pause
 exit /b 1
)
"%VEX_TOOLS%\tools\bin\make.exe" "P=ball-serial" "T=%VEX_BASE%\sdk\cpp\IQ2\IQ2_20250520_15_00_00"
if errorlevel 1 (
 echo Build failed. Copy the error above.
 pause
 exit /b 1
)
echo Build complete. This command does not download or run anything on the Brain.
pause
