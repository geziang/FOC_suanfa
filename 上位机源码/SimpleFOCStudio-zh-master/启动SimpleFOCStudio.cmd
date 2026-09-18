@echo off
setlocal
cd /d "%~dp0"

if not exist ".venv\Scripts\python.exe" (
    echo Virtual environment not found. Configuring it now...
    powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0配置SimpleFOCStudio环境.ps1"
    if errorlevel 1 (
        echo Environment setup failed.
        pause
        exit /b 1
    )
)

powershell.exe -NoProfile -ExecutionPolicy Bypass -File "%~dp0启动SimpleFOCStudio.ps1"
if errorlevel 1 (
    echo SimpleFOC Studio failed to start.
    pause
    exit /b 1
)
endlocal
