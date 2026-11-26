@echo off
setlocal
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
 echo Run 1 - Setup.cmd first.
 pause
 exit /b 1
)
".venv\Scripts\python.exe" -c "from start_phone_camera import find_cloudflared; import sys; sys.exit(0 if find_cloudflared() else 1)"
if not errorlevel 1 goto ready
winget install --id Cloudflare.cloudflared --exact --source winget --scope user --silent --accept-package-agreements --accept-source-agreements --disable-interactivity
if errorlevel 1 (
 echo Cloudflare installation did not finish. Copy the error above.
 pause
 exit /b 1
)
:ready
echo Ready. Run 11 - Pair phone camera.cmd to open your pairing QR code.
pause
