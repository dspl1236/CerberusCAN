# Building the single CerberusConsole.exe

Produces one self-contained Windows app — Console + both firmware hexes +
the `teensy_loader_cli` flasher — that an end user just double-clicks. **No
Python or extra tools needed on their machine.**

> **You usually don't need to build this yourself.** GitHub Actions
> (`.github/workflows/build-console.yml`) auto-builds the `.exe` on every
> `CONSOLE_VERSION` bump and publishes it to the **Releases** page (compiling
> `teensy_loader_cli` in CI so the flasher is bundled). Grab it there. Build
> locally only when iterating on the app. The steps below are that local path.

## One-time setup (build machine only)
1. `pip install pyinstaller`
2. Put **`teensy_loader_cli.exe`** in `host\tools\` (see `tools\README.txt` —
   build it from PaulStoffregen/teensy_loader_cli, or drop in a prebuilt copy).
   *(Optional but recommended for distribution: also save its GPLv3 license as
   `host\tools\LICENSE-teensy_loader_cli.txt`.)*

## Build
```
cd host
build_exe.bat
```
→ `host\dist\CerberusConsole.exe` (single file).

The script bundles `firmware\cerberus-can-teensy40.hex` + `…teensy41.hex` and,
if present, `tools\teensy_loader_cli.exe`. It builds fine without the flasher —
the Flash button just reports "loader not found" until you add it.

## Notes
- **one-file** (`--onefile`) is the default here = one `.exe`. If antivirus/SmartScreen
  flags it, switch to one-dir (`--onedir`) + an Inno Setup installer, or code-sign the exe.
- The app finds its bundled files via `sys._MEIPASS` when frozen (handled in
  `cerberus_console.py` `_base()`), so the hexes/loader resolve inside the exe.
- Keep `CONSOLE_VERSION` / `BUNDLED_FW` in `cerberus_console.py` in lockstep with
  the firmware version when you refresh the bundled hexes.
