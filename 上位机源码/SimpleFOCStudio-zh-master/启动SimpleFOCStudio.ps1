$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location -LiteralPath $projectRoot

$venvPython = Join-Path $projectRoot '.venv\Scripts\python.exe'
$entryPoint = Join-Path $projectRoot 'simpleFOCStudio.py'

if (-not (Test-Path -LiteralPath $venvPython)) {
    Write-Host 'Virtual environment not found. Configuring it now...'
    & (Join-Path $projectRoot '配置SimpleFOCStudio环境.ps1')
}

if (-not (Test-Path -LiteralPath $venvPython)) {
    throw 'Virtual environment creation failed.'
}

& $venvPython -c "import PyQt5, pyqtgraph, serial, numpy"
if ($LASTEXITCODE -ne 0) {
    Write-Host 'Dependency check failed. Repairing the environment...'
    & (Join-Path $projectRoot '配置SimpleFOCStudio环境.ps1')
}

Write-Host 'Starting SimpleFOC Studio...'
& $venvPython -u $entryPoint
exit $LASTEXITCODE
