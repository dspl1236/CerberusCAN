Drop teensy_loader_cli.exe in THIS folder to enable the Console's one-click
"Flash / Update firmware" button (and to have build_exe.bat bundle it into the
single .exe).

Where to get it:
  - Build from source: https://github.com/PaulStoffregen/teensy_loader_cli
      (Windows: install MinGW-w64, then  make OS=WINDOWS  -> teensy_loader_cli.exe)
  - Or grab a prebuilt copy (PJRC loader_cli page, or a Teensyduino install's
    hardware\tools\ folder).

LICENSE: teensy_loader_cli is GPLv3. It is invoked as a separate process (not
linked), so it does not affect the Console's own license — but if you DISTRIBUTE
the binary, include its GPLv3 license text (save it here as
LICENSE-teensy_loader_cli.txt) and point to the source repo above.
