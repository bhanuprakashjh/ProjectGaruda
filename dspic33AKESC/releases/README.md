# Validated firmware binaries

| File | Source commit | Config | Validation |
|---|---|---|---|
| `garuda_am32_233k_20260612.hex` | `7e96604` | MOTOR_PROFILE=2 (2810), FEATURE_AM32_STARTUP=1 | Full-range bench 2026-06-12: AM32 kick+listen startup, 10.4k idle, sweep to 233,535 eRPM peak @ 98% duty, rej 2%, clean decel, zero faults (buildHash 4188998252) |

Rebuildable from the source commit with a clean MPLAB build; kept here so the
exact validated binary is distributable without a toolchain.
