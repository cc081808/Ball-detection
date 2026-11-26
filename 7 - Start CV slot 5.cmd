@echo off
cd /d "%~dp0"
".venv\Scripts\python.exe" cv_autonomy.py --camera phone
pause
