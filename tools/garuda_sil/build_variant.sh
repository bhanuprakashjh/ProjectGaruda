#!/usr/bin/env bash
# build_variant.sh NAME sed_expr [sed_expr...] — build a flag-variant .so.
# Temporarily edits dspic33AKESC/garuda_config.h (restored via git on exit),
# exactly mirroring the bench flag-experiment workflow, flash-free.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
FW="$(cd "$HERE/../../dspic33AKESC" && pwd)"
NAME="$1"; shift
cp "$FW/garuda_config.h" /tmp/garuda_config_h.bak
trap 'cp /tmp/garuda_config_h.bak "$FW/garuda_config.h"' EXIT
for e in "$@"; do sed -i "$e" "$FW/garuda_config.h"; done
grep -n "FEATURE_SKIP_MORPH\s\|FEATURE_CL_COAST_VERIFY\s" "$FW/garuda_config.h" | head -4
SIL_OUT="$HERE/libgaruda_sil_$NAME.so" bash "$HERE/build.sh"
