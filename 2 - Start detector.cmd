@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
 echo Run 1 - Setup first.
 pause
 exit /b 1
)
".venv\Scripts\python.exe" detect_ball.py
pause
