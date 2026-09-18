param(
    [switch]$Force
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location -LiteralPath $projectRoot

$venvPath = Join-Path $projectRoot '.venv'
$venvPython = Join-Path $venvPath 'Scripts\python.exe'
$requirements = Join-Path $projectRoot 'requirements-windows.txt'

if (-not (Get-Command uv -ErrorAction SilentlyContinue)) {
    throw 'uv was not found. Install uv, then run this script again.'
}

if ($Force -and (Test-Path -LiteralPath $venvPath)) {
    Remove-Item -LiteralPath $venvPath -Recurse -Force
}

if (-not (Test-Path -LiteralPath $venvPython)) {
    Write-Host 'Creating Python 3.10 virtual environment...'
    & uv venv $venvPath --python 3.10
}

Write-Host 'Installing SimpleFOC Studio dependencies...'
& uv pip install --python $venvPython -r $requirements

Write-Host 'Checking dependencies...'
& $venvPython -c "import PyQt5, pyqtgraph, serial, numpy; print('SimpleFOC Studio dependency check passed')"

Write-Host ''
Write-Host 'Environment ready. Double-click Start-SimpleFOCStudio.cmd to launch it.'
