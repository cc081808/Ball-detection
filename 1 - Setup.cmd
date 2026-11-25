@echo off
cd /d "%~dp0"
if exist ".venv\Scripts\python.exe" goto install
py -3.12 -m venv .venv
if errorlevel 1 (
 echo Python 3.12 is unavailable. Trying the installed Python 3 version.
 py -3 -m venv .venv
)
if errorlevel 1 goto failed
:install
".venv\Scripts\python.exe" --version
".venv\Scripts\python.exe" -m pip install -r requirements.txt
if errorlevel 1 goto failed
".venv\Scripts\python.exe" detect_ball.py --check
if errorlevel 1 goto failed
echo Setup and model check completed successfully.
pause
exit /b 0
:failed
echo Setup did not complete. See the error above and README.md.
pause
exit /b 1
