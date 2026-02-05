#!/usr/bin/env python3
"""
Simple test for Faultier Cat serial passthrough (UART monitor)

This script demonstrates reading target UART via Faultier's passthrough.
"""

import time
import sys
from faultier_controller import FaultierController


def main():
    print("=" * 50)
    print("Faultier Cat Serial Monitor Test")
    print("=" * 50)

    ctrl = FaultierController()

    try:
        # Connect to Faultier
        print("\n1. Connecting to Faultier...")
        success, msg = ctrl.connect_faultier()
        print(f"   {msg}")
        if not success:
            return 1

        serial_path = ctrl.get_faultier_serial_path()
        print(f"   Serial path: {serial_path}")

        # Connect to target via Faultier
        print("\n2. Connecting to target via Faultier Cat...")
        success, msg = ctrl.connect_target_via_faultier(baudrate=9600)
        print(f"   {msg}")
        if not success:
            return 1

        # Monitor for 5 seconds
        print("\n3. Monitoring target UART for 5 seconds...")
        print("-" * 50)
        start = time.time()
        line_count = 0
        while time.time() - start < 5:
            line = ctrl._read_target_line(timeout=0.1)
            if line:
                print(line)
                line_count += 1
        print("-" * 50)
        print(f"   Received {line_count} lines in 5 seconds")

        # Power cycle and monitor startup
        print("\n4. Power cycling and watching startup...")
        ctrl.power_cycle_target()

        print("-" * 50)
        start = time.time()
        while time.time() - start < 3:
            line = ctrl._read_target_line(timeout=0.1)
            if line:
                print(line)
        print("-" * 50)

    except KeyboardInterrupt:
        print("\n\nInterrupted")
    finally:
        print("\n5. Cleanup...")
        ctrl.cleanup()

    print("Done")
    return 0


if __name__ == "__main__":
    sys.exit(main())
