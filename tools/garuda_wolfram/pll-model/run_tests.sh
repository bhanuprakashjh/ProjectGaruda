#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")"
fail=0
for t in tests/test_*.wl; do
  echo "=== $t ==="
  if ! wolframscript -file "$t"; then fail=1; fi
done
exit $fail
