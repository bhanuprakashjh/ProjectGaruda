#!/usr/bin/env bash
# Install a clickable desktop launcher for garuda-gui (Linux).
# Usage: ./scripts/install_desktop_entry.sh
set -e
HERE="$(cd "$(dirname "$0")/.." && pwd)"
VENV_BIN="$(cd "$HERE/../../venv/bin" && pwd)"
APPS="$HOME/.local/share/applications"
mkdir -p "$APPS"
DESKTOP="$APPS/garuda-gui.desktop"
cat > "$DESKTOP" << EOD
[Desktop Entry]
Type=Application
Name=Garuda ESC GUI
Comment=Garuda ESC monitor, tuner and burst scope
Exec=$VENV_BIN/garuda-gui
Path=$HERE
Icon=$HERE/assets/garuda.svg
Terminal=false
Categories=Development;Electronics;
EOD
chmod +x "$DESKTOP"
# put a copy on the Desktop too, if one exists
DESKDIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
if [ -d "$DESKDIR" ]; then
  cp "$DESKTOP" "$DESKDIR/"
  chmod +x "$DESKDIR/garuda-gui.desktop"
  gio set "$DESKDIR/garuda-gui.desktop" metadata::trusted true 2>/dev/null || true
fi
update-desktop-database "$APPS" 2>/dev/null || true
echo "Installed: $DESKTOP"
[ -d "$DESKDIR" ] && echo "Desktop copy: $DESKDIR/garuda-gui.desktop"
echo "GNOME note: if the desktop icon shows 'untrusted', right-click it once -> Allow Launching."
