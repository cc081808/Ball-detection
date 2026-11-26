@echo off
cd /d "%~dp0"
call code --new-window "%~dp0vex-receiver"
if errorlevel 1 (
 echo Open the vex-receiver folder in VS Code manually.
 pause
)
