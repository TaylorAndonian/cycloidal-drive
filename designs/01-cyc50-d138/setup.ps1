# setup.ps1 -- recreate the venv in this folder on Windows and rerun the whole pipeline.
#   Right-click > Run with PowerShell, or:  powershell -ExecutionPolicy Bypass -File .\setup.ps1
# Needs Python 3.10+ on PATH (python.org installer; build123d wheels exist for 3.10-3.13).
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot
if (-not (Test-Path ".venv")) { python -m venv .venv }
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe scripts\cad.py
.\.venv\Scripts\python.exe scripts\analysis.py
.\.venv\Scripts\python.exe scripts\check_geometry.py
.\.venv\Scripts\python.exe scripts\render_section.py
Write-Host "done: see cad\, analysis\, renders\"
