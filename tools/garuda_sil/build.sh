#!/usr/bin/env bash
# build.sh — compile the REAL dsPIC33AK ESC firmware core for x86 behind the
# mock HAL, producing libgaruda_sil.so.
#
# -m64: dsPIC33AK int=32 and the firmware uses stdint types throughout;
#       x86-64 acceptable for M1 (see DESIGN.md). -m32 multilib not assumed
#       to be installed.
set -euo pipefail

HERE="$(cd "$(dirname "$0")" && pwd)"
FW="$(cd "$HERE/../../dspic33AKESC" && pwd)"
OUT="${SIL_OUT:-$HERE/libgaruda_sil.so}"

CFLAGS=(
    -std=gnu11 -m64 -fPIC -shared -g -O1
    -DSIL_BUILD
    -Wall -Wno-attributes -Wno-unknown-pragmas -Wno-comment
    -fno-strict-aliasing
    -I "$HERE/mock"
    -I "$FW"
)

SRC=(
    "$FW/garuda_service.c"
    "$FW/motor/commutation.c"
    "$FW/motor/bemf_zc.c"
    "$FW/motor/startup.c"
    "$FW/motor/hwzc.c"
    "$FW/motor/speed_pi.c"
    "$FW/gsp/gsp_params.c"
    "$HERE/sil/virtual_hw.c"
    "$HERE/sil/hal_sil.c"
    "$HERE/sil/stubs.c"
    "$HERE/sil/sil_api.c"
)

echo "FW tree : $FW"
echo "Output  : $OUT"
gcc "${CFLAGS[@]}" -I "$HERE/sil" "${SRC[@]}" -o "$OUT" -lm
echo "OK: $OUT"
