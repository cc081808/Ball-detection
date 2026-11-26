@echo off
cd /d "%~dp0.."
".venv\Scripts\python.exe" update_cv_receiver.py --build-only
pause
