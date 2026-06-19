@echo off
REM ============================================================================
REM  Build CerberusConsole.exe  — a single self-contained Windows app
REM  (Console + both firmware hexes + teensy_loader_cli flasher, no Python needed).
REM
REM  Prereqs on THIS (build) machine only:
REM     pip install pyinstaller
REM     tools\teensy_loader_cli.exe   (build from PaulStoffregen/teensy_loader_cli, or download)
REM  The end user needs NOTHING — they just run dist\CerberusConsole.exe.
REM ============================================================================
cd /d "%~dp0"

set ADD=--add-data "..\firmware\cerberus-can-teensy41.hex;firmware" --add-data "..\firmware\cerberus-can-teensy40.hex;firmware"
set ADD=%ADD% --add-data "didb;didb" --add-data "saedb;saedb"

if exist "tools\teensy_loader_cli.exe" (
  set ADD=%ADD% --add-binary "tools\teensy_loader_cli.exe;tools"
) else (
  echo [warn] tools\teensy_loader_cli.exe not found — building WITHOUT the flasher.
  echo        Drop it in host\tools\ and re-run to get one-click firmware updates.
)
if exist "tools\LICENSE-teensy_loader_cli.txt" set ADD=%ADD% --add-data "tools\LICENSE-teensy_loader_cli.txt;tools"

pyinstaller --noconfirm --onefile --windowed --name CerberusConsole %ADD% cerberus_console.py

echo.
echo === Done. Single app: dist\CerberusConsole.exe ===
