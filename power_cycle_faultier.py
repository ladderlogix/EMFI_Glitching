#!/usr/bin/env python3
"""
Simple power cycle script for Faultier.

This script is designed to be called via subprocess to avoid thread/signal issues.
It opens Faultier, does the power cycle, and exits.

Usage:
    python3 power_cycle_faultier.py [OUTPUT] [LENGTH_MS]

    OUTPUT: MUX0, MUX1, MUX2, EXT0, EXT1, CROWBAR (default: MUX0)
    LENGTH_MS: Power cycle length in milliseconds (default: 100)

Exit codes:
    0 = Success
    1 = Error
"""

import sys
import time

def main():
    # Parse arguments
    output_name = sys.argv[1] if len(sys.argv) > 1 else "MUX0"
    length_ms = int(sys.argv[2]) if len(sys.argv) > 2 else 500  # Default 500ms for more reliable reset

    output_name = output_name.upper()

    try:
        from faultier import Faultier
        import faultier as faultier_mod

        # Map output name to constant
        output_map = {
            "CROWBAR": faultier_mod.OUT_CROWBAR,
            "MUX0": faultier_mod.OUT_MUX0,
            "MUX1": faultier_mod.OUT_MUX1,
            "MUX2": faultier_mod.OUT_MUX2,
            "EXT0": faultier_mod.OUT_EXT0,
            "EXT1": faultier_mod.OUT_EXT1,
        }

        if output_name not in output_map:
            print(f"ERROR: Unknown output '{output_name}'", file=sys.stderr)
            print(f"Valid outputs: {', '.join(output_map.keys())}", file=sys.stderr)
            return 1

        output_val = output_map[output_name]

        # Convert ms to clock cycles (100MHz clock)
        # 1ms = 100,000 cycles
        power_cycle_length = length_ms * 100000

        print(f"Connecting to Faultier...", file=sys.stderr)
        f = Faultier()
        print(f"Connected. Configuring power cycle: output={output_name} ({output_val}), length={length_ms}ms ({power_cycle_length} cycles)", file=sys.stderr)

        # Configure and execute power cycle
        f.configure_glitcher(
            power_cycle_output=output_val,
            power_cycle_length=power_cycle_length,
        )

        print(f"Executing power cycle...", file=sys.stderr)
        f.power_cycle()
        print(f"Power cycle complete.", file=sys.stderr)

        # Reset to safe state
        f.default_settings()

        print(f"OK: Power cycled via {output_name} for {length_ms}ms")
        return 0

    except Exception as e:
        print(f"ERROR: {e}", file=sys.stderr)
        import traceback
        traceback.print_exc(file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
