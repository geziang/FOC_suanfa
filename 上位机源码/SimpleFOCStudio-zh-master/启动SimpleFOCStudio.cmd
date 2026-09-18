@echo off
setlocal
set "PROJECT_ROOT=%~dp0"
powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%PROJECT_ROOT%启动SimpleFOCStudio.ps1"
if errorlevel 1 (
    echo SimpleFOC Studio failed to start.
    pause
    exit /b 1
)
endlocal
