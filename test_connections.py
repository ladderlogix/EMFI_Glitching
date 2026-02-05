#!/usr/bin/env python3
"""
Test script to verify all device connections for EMFI setup.

Devices:
- Faultier: Power cycling and glitch timing
- Faulty Cat: EMP pulse generator
- Target UART: Via TI debug probe or other serial adapter
"""

import serial
import time
import sys

def test_serial_port(port, baud=9600, description=""):
    """Test a serial port and read any available data"""
    print(f"\n{'='*50}")
    print(f"Testing: {port} ({description})")
    print(f"Baud rate: {baud}")
    print('='*50)

    try:
        ser = serial.Serial(port, baud, timeout=0.5)
        time.sleep(0.2)

        print("Reading for 2 seconds...")
        start = time.time()
        data = b''
        while time.time() - start < 2:
            if ser.in_waiting:
                chunk = ser.read(ser.in_waiting)
                data += chunk
                print(f"  Received: {chunk[:50]}..." if len(chunk) > 50 else f"  Received: {chunk}")
            time.sleep(0.05)

        ser.close()

        if data:
            print(f"\nTotal bytes received: {len(data)}")
            return True
        else:
            print("\nNo data received")
            return False

    except serial.SerialException as e:
        print(f"Error: {e}")
        return False


def test_faultier():
    """Test Faultier connection"""
    print("\n" + "="*60)
    print("TESTING FAULTIER")
    print("="*60)

    try:
        from faultier import Faultier
        f = Faultier()
        f.default_settings()

        serial_path = f.get_serial_path()
        print(f"Faultier found!")
        print(f"Serial passthrough: {serial_path}")

        return True, serial_path
    except Exception as e:
        print(f"Faultier error: {e}")
        return False, None


def test_faultycat(port='/dev/ttyACM1', baud=115200):
    """Test Faulty Cat connection"""
    print("\n" + "="*60)
    print("TESTING FAULTY CAT")
    print("="*60)

    try:
        ser = serial.Serial(port, baud, timeout=1)
        time.sleep(0.3)

        # Send disarm command
        ser.write(b'd\r\n')
        time.sleep(0.1)

        # Read response
        if ser.in_waiting:
            response = ser.read(ser.in_waiting)
            print(f"Response to 'd': {response}")

        ser.close()
        print(f"Faulty Cat found at {port}")
        return True
    except Exception as e:
        print(f"Faulty Cat error: {e}")
        return False


def main():
    print("="*60)
    print("EMFI Connection Tester")
    print("="*60)

    # List available ports
    import glob
    ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
    print(f"\nAvailable serial ports: {ports}")

    # Check by-id links
    by_id = glob.glob('/dev/serial/by-id/*')
    if by_id:
        print("\nDevice identification:")
        for path in by_id:
            import os
            target = os.readlink(path)
            print(f"  {path} -> {target}")

    # Test Faultier
    faultier_ok, faultier_serial = test_faultier()

    # Test Faulty Cat
    faultycat_ok = test_faultycat()

    # Test serial ports for target UART
    print("\n" + "="*60)
    print("TESTING SERIAL PORTS FOR TARGET UART")
    print("="*60)

    for port in ports:
        if 'Faultier' not in port and 'Faulty' not in port:
            test_serial_port(port, 9600, "Target UART candidate")

    # If Faultier available, test its serial passthrough
    if faultier_ok and faultier_serial:
        test_serial_port(faultier_serial, 9600, "Faultier UART passthrough")

    # Summary
    print("\n" + "="*60)
    print("SUMMARY")
    print("="*60)
    print(f"Faultier: {'OK' if faultier_ok else 'NOT FOUND'}")
    print(f"Faulty Cat: {'OK' if faultycat_ok else 'NOT FOUND'}")
    print("\nIf target UART shows no data, try:")
    print("  1. Check physical connections")
    print("  2. Verify target is powered and running")
    print("  3. Try different baud rates (9600, 115200)")
    print("  4. Power cycle the target board")


if __name__ == "__main__":
    main()
