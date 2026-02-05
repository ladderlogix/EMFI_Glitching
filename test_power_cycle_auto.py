#!/usr/bin/env python3
"""
Automatic test - pulses each Faultier output in sequence.
Watch your target board to see which output causes it to reset.

No user interaction needed - just watch the board!
"""

from faultier import Faultier
import faultier as faultier_mod
import time
import sys

def main():
    print("=" * 60)
    print("AUTOMATIC FAULTIER OUTPUT TEST")
    print("=" * 60)
    print("\nThis will pulse each output for 100ms with 2 second gaps.")
    print("Watch your target board to see which output causes a reset!\n")

    # Connect
    print("Connecting to Faultier...")
    try:
        f = Faultier()
        f.default_settings()
        print("Connected!\n")
    except Exception as e:
        print(f"ERROR: Could not connect: {e}")
        print("Unplug/replug Faultier and try again.")
        return 1

    outputs = [
        ("CROWBAR", faultier_mod.OUT_CROWBAR),
        ("MUX0", faultier_mod.OUT_MUX0),
        ("MUX1", faultier_mod.OUT_MUX1),
        ("MUX2", faultier_mod.OUT_MUX2),
        ("EXT0", faultier_mod.OUT_EXT0),
        ("EXT1", faultier_mod.OUT_EXT1),
    ]

    print("Starting in 3 seconds... WATCH THE BOARD!")
    time.sleep(3)

    for name, output_val in outputs:
        print(f"\n>>> PULSING {name} (output={output_val})...", flush=True)

        try:
            f.configure_glitcher(
                power_cycle_output=output_val,
                power_cycle_length=10000000,  # ~100ms at 100MHz
            )

            f.power_cycle()
            print(f"    {name} pulsed!")

            f.default_settings()

            print("    Waiting 2 seconds...")
            time.sleep(2)

        except Exception as e:
            print(f"    ERROR on {name}: {e}")
            # Reconnect
            try:
                time.sleep(1)
                f = Faultier()
                f.default_settings()
                print("    Reconnected")
            except:
                print("    Could not reconnect!")
                break

    print("\n" + "=" * 60)
    print("TEST COMPLETE")
    print("=" * 60)
    print("\nWhich output caused your board to reset?")
    print("Use that output name in the GUI's 'Power Output' dropdown.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
