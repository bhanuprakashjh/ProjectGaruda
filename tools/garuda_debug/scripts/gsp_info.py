#!/usr/bin/env python3
"""garuda-gsp-info — identity + sanity check over GSP (replaces check_max_erpm).

Prints firmware/build/profile/feature identity and flags common config traps
(EEPROM-vs-code drift, Vbus vs profile, maxErpm vs KV) at a glance.
"""
import argparse
import sys

from garuda_gsp import GspClient, GspError, list_ports_human


def main():
    ap = argparse.ArgumentParser(description="GSP identity / sanity check")
    ap.add_argument("--port", default=None, help="serial port (default: auto-detect)")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--list-ports", action="store_true")
    args = ap.parse_args()

    if args.list_ports:
        for dev, desc in list_ports_human():
            print(f"  {dev}  {desc}")
        return 0

    try:
        with GspClient(args.port, args.baud) as c:
            if not c.ping():
                print("FAIL: no PING response (port/baud/power?)")
                return 2
            info = c.get_info()
            snap = c.get_snapshot()
            print(f"  port           : {c.port}")
            print(f"  protocol ver   : {info['protocolVersion']}  (INFO {info['infoBytes']}B)")
            print(f"  firmware       : v{info['fwVersion']}  build="
                  f"{('0x%08X' % info['buildHash']) if info['buildHash'] else 'n/a'}")
            print(f"  board id       : 0x{info['boardId']:04X}")
            print(f"  motor profile  : {info['motorProfile']} (id {info['motorProfileId']})")
            print(f"  pole pairs     : {info['motorPolePairs']}")
            print(f"  mode           : {'FOC' if info['isFoc'] else '6-step'}  "
                  f"(features 0x{info['featureFlags']:08X})")
            print(f"  PWM freq       : {info['pwmFrequency']:,} Hz")
            print(f"  maxErpm cap    : {info['maxErpm']:,}")
            print(f"  live           : state={snap['state_name']} "
                  f"Vbus={snap['vbus_V']:.1f}V fault={snap['fault_name']}")

            # quick sanity flags
            warns = []
            if snap["vbus_V"] > 30:
                warns.append(f"Vbus {snap['vbus_V']:.1f}V is high — confirm cell count vs motor rating")
            if info["maxErpm"] < 50_000:
                warns.append(f"maxErpm cap {info['maxErpm']:,} is low — may clip top speed")
            if warns:
                print("  flags:")
                for w in warns:
                    print(f"    ⚠ {w}")
            return 0
    except (GspError, OSError) as e:
        print(f"ERROR: {e}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
