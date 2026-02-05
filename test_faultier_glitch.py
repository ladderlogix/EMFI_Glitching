#!/usr/bin/env python3
"""
Test script for Faultier glitching with UART passthrough (Faultier Cat)

This script demonstrates:
1. Connecting to Faultier
2. Connecting to target via Faultier's UART passthrough
3. Power cycling the target
4. Performing glitch attempts
"""

import time
import sys
from faultier_controller import FaultierController, TargetState, GlitchResult


def main():
    print("=" * 60)
    print("Faultier Glitch Test with UART Passthrough")
    print("=" * 60)

    # Create controller
    ctrl = FaultierController()

    # Set up message logging
    def on_message(msg):
        print(f"[LOG] {msg}")

    def on_state_change(state):
        print(f"[STATE] Target state: {state.name}")

    ctrl.on_message = on_message
    ctrl.on_state_change = on_state_change

    try:
        # Connect to Faultier
        print("\n1. Connecting to Faultier...")
        success, msg = ctrl.connect_faultier()
        print(f"   {msg}")
        if not success:
            print("   FAILED: Could not connect to Faultier")
            return 1

        # Get serial path
        serial_path = ctrl.get_faultier_serial_path()
        print(f"   Serial passthrough: {serial_path}")

        # Connect to target via Faultier
        print("\n2. Connecting to target via Faultier UART passthrough...")
        success, msg = ctrl.connect_target_via_faultier(baudrate=9600)
        print(f"   {msg}")
        if not success:
            print("   FAILED: Could not connect to target")
            return 1

        # Read current state
        print("\n3. Reading target output for 2 seconds...")
        start = time.time()
        while time.time() - start < 2:
            line = ctrl._read_target_line(timeout=0.2)
            if line:
                print(f"   TARGET: {line}")

        # Power cycle test
        print("\n4. Power cycling target...")
        success, msg = ctrl.power_cycle_target()
        print(f"   {msg}")

        # Wait and read output after power cycle
        print("\n5. Reading target output after power cycle...")
        start = time.time()
        while time.time() - start < 3:
            line = ctrl._read_target_line(timeout=0.2)
            if line:
                print(f"   TARGET: {line}")

        # Configure glitcher
        print("\n6. Configuring glitcher...")
        ctrl.set_glitch_parameters(delay_ns=1000, pulse_ns=100)
        success, msg = ctrl.configure_faultier_trigger(
            trigger_type="RISING_EDGE",
            trigger_source="EXT0",
            glitch_output="CROWBAR"
        )
        print(f"   {msg}")

        # Perform a few glitch attempts
        print("\n7. Performing glitch attempts...")
        for i in range(3):
            print(f"\n   Attempt {i+1}:")
            attempt = ctrl.perform_glitch_attempt(0, 0, 0)
            print(f"   Result: {attempt.result.name}")
            print(f"   Heartbeats: {attempt.heartbeats_received}")
            print(f"   Response time: {attempt.response_time_ms:.1f}ms")
            time.sleep(0.5)

        # Summary
        print("\n" + "=" * 60)
        print("Test Summary")
        print("=" * 60)
        stats = ctrl.get_statistics()
        print(f"Total attempts: {stats['total_attempts']}")
        print(f"Glitches: {stats['total_glitches']} ({stats['glitch_rate']*100:.1f}%)")
        print(f"Crashes: {stats['total_crashes']} ({stats['crash_rate']*100:.1f}%)")
        print(f"Power cycles: {stats['total_power_cycles']}")

    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    finally:
        print("\n8. Cleaning up...")
        ctrl.cleanup()
        print("   Done")

    return 0


if __name__ == "__main__":
    sys.exit(main())
