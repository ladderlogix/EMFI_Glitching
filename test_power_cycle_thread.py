#!/usr/bin/env python3
"""
Test power cycling from a background thread (like the GUI scan does).

This tests the _power_cycle_fallback method that bypasses the @no_interrupt
decorator which requires signals (main thread only).
"""

from faultier import Faultier
import faultier as faultier_mod
import threading
import time
import sys

def power_cycle_in_thread(power_output="MUX0"):
    """This runs in a background thread, simulating what the GUI scan does."""
    print(f"\n[Thread] Starting power cycle test with output: {power_output}")

    try:
        # Import the protobuf classes
        from faultier.faultier_pb2 import Command, CommandGlitch, CommandConfigureGlitcher

        # Connect to Faultier
        print("[Thread] Connecting to Faultier...")
        f = Faultier()
        print("[Thread] Connected!")

        # Map output name to constant
        output_map = {
            "CROWBAR": faultier_mod.OUT_CROWBAR,
            "MUX0": faultier_mod.OUT_MUX0,
            "MUX1": faultier_mod.OUT_MUX1,
            "MUX2": faultier_mod.OUT_MUX2,
            "EXT0": faultier_mod.OUT_EXT0,
            "EXT1": faultier_mod.OUT_EXT1,
        }
        output_val = output_map.get(power_output, faultier_mod.OUT_MUX0)

        # power_cycle_length is in CLOCK CYCLES (100MHz)
        # 100ms = 10,000,000 cycles
        power_cycle_length = 10000000

        print(f"[Thread] Configuring power cycle: output={output_val}, length={power_cycle_length}")

        # Create config (mimics what Faultier.power_cycle() does internally)
        config = CommandConfigureGlitcher(
            trigger_type=faultier_mod.TRIGGER_NONE,
            trigger_source=faultier_mod.TRIGGER_IN_NONE,
            glitch_output=faultier_mod.OUT_NONE,
            delay=0,
            pulse=0,
            power_cycle_output=output_val,
            power_cycle_length=power_cycle_length,
            trigger_pull_configuration=faultier_mod.TRIGGER_PULL_NONE
        )

        # Send configuration
        print("[Thread] Sending configuration...")
        f._send_configuration(config)

        # Send glitch command to trigger the power cycle
        print("[Thread] Sending power cycle command...")
        cmd = Command()
        cmd.glitch.CopyFrom(CommandGlitch())
        f._send_protobuf(cmd)

        # Wait for response
        print("[Thread] Waiting for response...")
        f._check_response()

        print("[Thread] Power cycle complete!")
        print("[Thread] Did your target board reset? (watch it!)")

        # Reset to defaults
        f.glitcher_configuration = f._get_default_settings()

        return True

    except Exception as e:
        print(f"[Thread] ERROR: {e}")
        import traceback
        traceback.print_exc()
        return False


def main():
    print("=" * 60)
    print("POWER CYCLE THREAD TEST")
    print("=" * 60)
    print("\nThis tests power cycling from a BACKGROUND THREAD")
    print("(which is how the GUI scan operates)")
    print()

    # Get output to test
    outputs = ["MUX0", "MUX1", "MUX2", "EXT0", "EXT1", "CROWBAR"]
    print("Available outputs:", ", ".join(outputs))

    choice = input("\nWhich output to test? [MUX0]: ").strip().upper()
    if not choice:
        choice = "MUX0"

    if choice not in outputs:
        print(f"Invalid choice: {choice}")
        return 1

    print(f"\nWill test {choice} from a background thread in 3 seconds...")
    print("WATCH YOUR TARGET BOARD!")
    time.sleep(3)

    # Run in a thread (like the GUI does)
    thread = threading.Thread(target=power_cycle_in_thread, args=(choice,))
    thread.start()
    thread.join(timeout=10)

    if thread.is_alive():
        print("\n[Main] Thread timed out!")
        return 1

    print("\n" + "=" * 60)
    print("TEST COMPLETE")
    print("=" * 60)
    print("\nIf your target reset, the fix is working!")
    print("If not, check your wiring or try a different output.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
