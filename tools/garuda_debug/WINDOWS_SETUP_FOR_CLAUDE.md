# Getting the Garuda GUI running on Windows — guide for Claude Code

**You (Claude) are running on Windows now.** The user booted/logged into Windows
and wants the `garuda-gui` debug tool working here — either run-from-source (fast,
recommended) or built into a standalone `.exe`. Follow this exactly, verifying each
step before the next. Announce which of the two paths you're taking.

There are two older docs in this folder:
- `INSTALL_WINDOWS.md` — end-user `uv` install **from GitHub**. DO NOT use it for
  this task: it pulls the repo default branch and will **miss the current work**
  (see Step 0). It's only for a clean end-user machine that wants the shipped version.
- `WINDOWS_BUILD_EXE.md` — the older exe recipe. This file supersedes it and fixes
  the icon-generation trap.

---

## Step 0 — CRITICAL: confirm you have the CURRENT source

The GUI lives in the **ProjectGaruda** repo. As of writing, the live code is on
branch **`ak512-port`**, which has **unpushed commits AND uncommitted working-tree
changes** to the host tooling (`garuda_gsp/client.py`, `decode.py`, `protocol.py`,
`session.py`, `garuda_gui/app.py`). These carry the STA-observer telemetry/decoder
work. A GitHub clone of the default branch will silently be **stale**.

**Verify you are on the right source before doing anything else.** From this folder
(`tools/garuda_debug/`):

```bat
py -c "import pathlib,sys; t=pathlib.Path('garuda_gsp/protocol.py').read_text(); sys.exit(0 if 'staK1bMilli' in t and 'staThetaBaseDegX10' in t else 1)" && echo SOURCE_OK || echo SOURCE_STALE
```

- `SOURCE_OK` → the STA params are present; you have current code. Proceed.
- `SOURCE_STALE` (or file-not-found) → **stop.** You have old/wrong source. Get the
  current tree over by ONE of:
  1. **(preferred) push then clone.** On the Linux side the user runs
     `git add -A && git commit -m wip && git push origin ak512-port`, then here:
     `git clone -b ak512-port git@github.com:bhanuprakashjh/ProjectGaruda.git`
     and `cd ProjectGaruda\tools\garuda_debug`.
  2. **copy the working tree.** Copy the whole `tools/garuda_debug/` folder from the
     Linux checkout (USB / network share) — this preserves uncommitted changes without
     pushing. Then re-run the verify command above.

Do not continue until you see `SOURCE_OK`.

---

## Prerequisites (verify, install if missing)

1. **Python 3.11 from python.org** — NOT the Microsoft Store build (its sandbox
   breaks PyInstaller and venv script paths). Check: `py -3.11 --version`. If absent,
   install from https://www.python.org/downloads/ and tick "Add python.exe to PATH".
2. Git (only if you cloned in Step 0). `git --version`.
3. You are in `tools/garuda_debug/` (contains `pyproject.toml`, `gui_launcher.py`).

---

## Path A — Run from source (recommended: fastest, uses YOUR current code)

This gets a working GUI in ~3 commands. No exe, no icon step, always matches the
source you verified in Step 0.

```bat
py -3.11 -m venv .venv-win
.venv-win\Scripts\activate
pip install -e .[gui]
```

Then smoke-test (plug the board in first):

```bat
garuda-gui
```

or equivalently `python gui_launcher.py`. Expected: the window opens, auto-starts its
serial broker, auto-detects the board's COM port (probes each with GET_INFO), and
streams telemetry. Every run auto-records to a `sessions\` folder in the launch dir.

Quick headless sanity check (separates "serial problem" from "GUI problem"):

```bat
garuda-gsp-info
```

prints firmware version / motor profile / pole pairs if the link works.

**If that all works, Path A is done** — the user has a working GUI. Only continue to
Path B if they specifically want a standalone `.exe` to hand to a machine with no
Python.

---

## Path B — Build the standalone `garuda-gui.exe`

Requires the Path A venv active. Adds PyInstaller.

```bat
.venv-win\Scripts\activate
pip install pyinstaller pillow
```

### B1 — the icon (make it non-blocking)

`garuda-gui.spec` references `assets\garuda.ico`, but only `assets\garuda.svg` is
committed. The old recipe used `cairosvg`, which needs native GTK/Cairo DLLs and
**frequently fails to install on Windows**. Do this instead — try the SVG→ICO
conversion, but never let it block the build:

```bat
:: Try cairosvg (best fidelity). If it fails to install or run, fall through.
pip install cairosvg 2>NUL && python -c "import cairosvg,io;from PIL import Image;png=cairosvg.svg2png(url='assets/garuda.svg',output_width=256);Image.open(io.BytesIO(png)).save('assets/garuda.ico',sizes=[(16,16),(32,32),(48,48),(256,256)])" 2>NUL && echo ICO_OK || echo ICO_SKIPPED
```

- `ICO_OK` → good, leave the spec as-is.
- `ICO_SKIPPED` → **edit `garuda-gui.spec`** and change the icon line in the `EXE(...)`
  block from `icon="assets/garuda.ico",` to `icon=None,`. The exe builds fine without
  a custom icon (it just gets the default PyInstaller icon). Do not spend time fighting
  cairo — the icon is cosmetic.

### B2 — smoke-test from source, then build

```bat
:: confirm the app runs from source first (close the window after it opens)
python gui_launcher.py
:: build
pyinstaller --noconfirm garuda-gui.spec
```

Result: `dist\garuda-gui\garuda-gui.exe` (onedir). Copy the whole `dist\garuda-gui\`
folder to any Windows machine — no Python needed there.

**How the one-exe/two-roles design works** (don't break it): `gui_launcher.py` is the
entry point. The GUI, when frozen, spawns its broker by re-exec'ing **itself** as
`garuda-gui.exe --broker` (see `garuda_gsp/broker.py: ensure_broker`, the
`sys.frozen` branch). So **do not rename the exe** — the child spawn relies on
`sys.executable` being this exe.

---

## Verify on the bench machine (both paths)

1. Plug the board in. If no COM port appears in Device Manager, install the USB-serial
   driver (PKoB4 / USB-CDC usually needs nothing on Win10+; a CH340/CP210x dongle needs
   its vendor driver).
2. Launch the GUI. It must find the COM port and connect.
3. Check: **telemetry streams**, the **Burst Scope** tab captures, **Esc** sends STOP,
   the **Params** tab reads/writes, **Diagnose** produces a report.

---

## Known Linux-only features (absent on Windows — this is expected, not a bug)

These are gated and fail gracefully with a message; they do NOT affect the core
telemetry / scope / params / diagnose workflow:

- **Lab → SPICE sim** (`_run_sim_spice`): needs a Linux `.venv-spice` with ngspice
  `.so` libs. On Windows it returns "SPICE venv not found" — ignore unless the user
  specifically wants circuit sim on Windows (out of scope here).
- **`garuda-gui --sim`** (SIL twin): needs `tools/garuda_sil/libgaruda_sil.so` (a Linux
  build). Not available on Windows; use a real board.

Everything else — live telemetry, state trace, param get/set, burst scope, session
auto-record, Claude diagnose — is pure Python and works on Windows.

---

## Troubleshooting (in order of likelihood)

- **"no ports found" / can't connect**: close **MPLAB X / Data Visualizer** — Windows
  COM ports are exclusive; a `[BUSY]` port is held by another program. Then close PuTTY/
  TeraTerm; then kill any leftover `garuda-broker`/`python` in Task Manager. Try another
  USB cable (charge-only cables exist) and port.
- **Never run under WSL** — WSL cannot see COM ports. Use native PowerShell/cmd.
- **Blank window / instant exit (exe)**: set `console=True` in `garuda-gui.spec`, rebuild,
  run from a terminal to see the traceback. Usually a missing PySide6 plugin →
  `pip install --force-reinstall PySide6` in the venv and rebuild.
- **Antivirus quarantines the exe**: PyInstaller false positive; whitelist the `dist`
  folder (UPX is already off in the spec).
- **`sessions\` location**: the app writes `sessions\` next to wherever it's STARTED
  from. For the exe, make a shortcut with "Start in" set to a writable folder.
- **Store-Python weirdness** (venv scripts missing, PyInstaller errors): you're on the
  Microsoft Store Python. Reinstall from python.org (see Prerequisites).
