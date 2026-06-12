# Garuda Debug GUI — Windows Install Guide

Installs the live debug GUI (telemetry, state trace, params, auto-recorded
sessions) on a Windows PC in two commands. No Python knowledge needed —
`uv` downloads everything, including Python itself.

## 1. Install uv (one time)

Open **PowerShell** and paste:

```powershell
powershell -ExecutionPolicy ByPass -c "irm https://astral.sh/uv/install.ps1 | iex"
```

Close and reopen PowerShell afterwards (so `uv` is on PATH).

## 2. Install the GUI (one time)

```powershell
uv tool install --python 3.11 "garuda-debug[gui] @ git+https://github.com/bhanuprakashjh/ProjectGaruda#subdirectory=tools/garuda_debug"
```

This fetches the latest code from GitHub and installs these commands:
`garuda-gui`, `garuda-broker`, `garuda-gsp-info`, `garuda-gsp-record`, `garuda-mcp`.

## 3. Run

Plug in the board (USB), then:

```powershell
garuda-gui
```

The GUI auto-starts its serial broker and auto-detects the board's COM port
(it probes each port with GET_INFO and picks whichever answers — the PKoB
debug COM is skipped automatically). Every run is auto-recorded to a
`sessions\` folder in the directory you launched from.

## Updating to the latest version

```powershell
uv tool upgrade garuda-debug
```

(or `uv tool install --reinstall ...` with the same URL as step 2).

## Troubleshooting

**No serial ports listed / can't connect** — see the "Windows serial-port
troubleshooting" section in `dspic33AKESC/docs/vex_motor_runbook.md`. Short
version, in order of likelihood:

1. Close **MPLAB X / Data Visualizer** — Windows COM ports are exclusive;
   a port shown as `[BUSY]` in the GUI's error message is held by another
   program.
2. Close any terminal app (PuTTY / TeraTerm).
3. Check Task Manager for a leftover `garuda-broker`/`python` process and
   end it.
4. No ports at all: try another USB cable (charge-only cables exist) and
   port; if using a CH340/CP210x dongle, install its driver.
5. Do **not** run anything under WSL — WSL cannot see COM ports. Use
   native PowerShell as above.

**`uv` not recognized after install** — reopen PowerShell, or log out/in.

**Corporate proxy blocks the install** — ask for the offline zip instead
(we can produce a self-contained folder that needs no downloads).

## Quick sanity check without the GUI

```powershell
garuda-gsp-info
```

prints the board identity (firmware version, motor profile, pole pairs) if
the connection works — useful to separate "serial problem" from "GUI problem".
