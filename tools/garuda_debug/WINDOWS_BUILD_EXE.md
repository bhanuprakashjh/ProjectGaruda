# Building garuda-gui.exe on Windows

This file is written to be handed to Claude Code on the Windows machine.
Open a terminal in THIS folder (`tools/garuda_debug/`), start `claude`, and say:

> "Build the Windows exe following WINDOWS_BUILD_EXE.md"

Claude: follow the steps below exactly, verifying each before the next.

## Prerequisites (verify, install if missing)
1. Python 3.11+ from python.org (NOT the Microsoft Store build — its sandbox
   breaks PyInstaller). `py --version` to check.
2. This repo checked out; you are in `tools/garuda_debug/`.

## Steps
```bat
py -m venv .venv-win
.venv-win\Scripts\activate
pip install -e .[gui]
pip install pyinstaller pillow
:: generate the .ico from the svg (PyInstaller needs .ico on Windows)
python -c "from PIL import Image; import cairosvg, io" 2>NUL || pip install cairosvg
python -c "import cairosvg,io;from PIL import Image;png=cairosvg.svg2png(url='assets/garuda.svg',output_width=256);Image.open(io.BytesIO(png)).save('assets/garuda.ico',sizes=[(16,16),(32,32),(48,48),(256,256)])"
:: smoke-test the app runs from source first (close the window after it opens)
python gui_launcher.py
:: build
pyinstaller --noconfirm garuda-gui.spec
```

Result: `dist\garuda-gui.exe` — single-folder app (`dist\garuda-gui\` if onedir;
the spec builds onefile-style EXE). Copy the whole `dist` output to any Windows
bench machine; no Python needed there.

## Verify on the bench machine
1. Plug the board in; install the USB-serial driver if the COM port doesn't
   appear in Device Manager (PKoB4/CDC usually needs nothing on Win10+).
2. Double-click `garuda-gui.exe` — it must find the COM port and connect.
3. Check: telemetry streams, Burst Scope tab captures, Esc sends STOP.

## Known trouble spots (fix in this order)
- **Blank window / instant exit**: rebuild with `console=True` in the spec to
  see the traceback; usually a missing PySide6 plugin -> `pip install --force
  PySide6` inside the venv and rebuild.
- **"no ports found"**: the broker child needs the same exe — the app spawns
  `garuda-gui.exe --broker` when frozen (gui_launcher.py handles this; do not
  rename the exe).
- **Antivirus quarantine**: PyInstaller false positive; whitelist the folder or
  build with `--noupx` (already off in the spec).
- **sessions/ output**: the exe writes `sessions\` next to wherever it is
  STARTED from; put a shortcut with "Start in" set to a writable folder.

## Optional: desktop shortcut on Windows
Right-click `garuda-gui.exe` -> Send to -> Desktop (create shortcut); set the
icon to `assets\garuda.ico` and "Start in" to the folder you want sessions in.
