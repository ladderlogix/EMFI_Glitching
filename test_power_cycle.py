#!/usr/bin/env python3
"""
Test script to find which Faultier output controls your target's power.

Run this script and watch your target board to see which output
causes it to reset/power cycle.
"""

from faultier import Faultier
import faultier as faultier_mod
import time
import sys

def main():
    print("=" * 60)
    print("FAULTIER POWER CYCLE TEST")
    print("=" * 60)
    print("\nThis will test each Faultier output to find which one")
    print("controls your target's power.\n")

    print("Faultier outputs:")
    print("  CROWBAR (0): Crowbar MOSFET gate - for voltage glitching")
    print("  MUX0 (1): Analogue switch ch0 (SMA connector)")
    print("  MUX1 (2): Analogue switch ch1 (20-pin header)")
    print("  MUX2 (3): Analogue switch ch2 (20-pin header)")
    print("  EXT0 (4): EXT0 header - external trigger")
    print("  EXT1 (5): EXT1 header - external trigger")
    print()

    # Connect
    print("Connecting to Faultier...")
    try:
        f = Faultier()
        f.default_settings()
        print("Connected!\n")
    except Exception as e:
        print(f"ERROR: Could not connect to Faultier: {e}")
        print("Make sure Faultier is plugged in and try unplugging/replugging it.")
        return 1

    outputs = [
        ("CROWBAR", faultier_mod.OUT_CROWBAR),
        ("MUX0", faultier_mod.OUT_MUX0),
        ("MUX1", faultier_mod.OUT_MUX1),
        ("MUX2", faultier_mod.OUT_MUX2),
        ("EXT0", faultier_mod.OUT_EXT0),
        ("EXT1", faultier_mod.OUT_EXT1),
    ]

    print("=" * 60)
    print("TESTING EACH OUTPUT")
    print("Watch your target board and Faultier LEDs!")
    print("=" * 60)

    for name, output_val in outputs:
        print(f"\n>>> Testing {name} (value={output_val})")
        input(f"    Press Enter to send 50ms pulse on {name}...")

        try:
            f.configure_glitcher(
                power_cycle_output=output_val,
                power_cycle_length=5000000,  # ~50ms at 100MHz clock
            )

            print(f"    Pulsing {name}...", end=" ", flush=True)
            f.power_cycle()
            print("Done!")

            response = input(f"    Did {name} affect your target? (y/n): ").strip().lower()
            if response == 'y':
                print(f"\n*** {name} controls your target power! ***")
                print(f"Use power_cycle_output={output_val} or '{name}' in the GUI.")

            f.default_settings()
            time.sleep(0.2)

        except Exception as e:
            print(f"    ERROR: {e}")
            print("    Reconnecting...")
            try:
                f = Faultier()
                f.default_settings()
            except:
                print("    Could not reconnect. Unplug/replug Faultier and restart.")
                return 1

    print("\n" + "=" * 60)
    print("TEST COMPLETE")
    print("=" * 60)

    # Also test longer pulse
    print("\nWould you like to test with a longer pulse (200ms)?")
    test_long = input("Enter output name (e.g., MUX0) or 'skip': ").strip().upper()

    if test_long != 'SKIP':
        output_map = {
            "CROWBAR": faultier_mod.OUT_CROWBAR,
            "MUX0": faultier_mod.OUT_MUX0,
            "MUX1": faultier_mod.OUT_MUX1,
            "MUX2": faultier_mod.OUT_MUX2,
            "EXT0": faultier_mod.OUT_EXT0,
            "EXT1": faultier_mod.OUT_EXT1,
        }

        if test_long in output_map:
            print(f"\nTesting {test_long} with 200ms pulse...")
            f.configure_glitcher(
                power_cycle_output=output_map[test_long],
                power_cycle_length=20000000,  # ~200ms
            )
            f.power_cycle()
            print("Done!")
            f.default_settings()

    print("\nDone. Use the working output in emfi_gui_faultier.py")
    return 0


if __name__ == "__main__":
    sys.exit(main())
