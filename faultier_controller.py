"""
Enhanced EMFI Controller with Faultier Integration

This controller provides:
- Proper Faultier library integration for glitching and power cycling
- State machine-based target communication parsing
- Automatic power cycling on crash/timeout detection
- Detailed glitch attempt statistics
"""

import serial
import time
import threading
import queue
import subprocess
import os
from enum import Enum, auto
from dataclasses import dataclass, field
from typing import Optional, Callable, List, Dict, Any
import numpy as np

# Try to import the Faultier library
try:
    from faultier import Faultier
    FAULTIER_AVAILABLE = True
except ImportError:
    FAULTIER_AVAILABLE = False
    print("Warning: Faultier library not available. Install with: pip install faultier")


class TargetState(Enum):
    """State machine states for target device communication"""
    UNKNOWN = auto()          # Initial/unknown state
    READY = auto()            # Target is ready for next attempt
    ATTEMPT_STARTED = auto()  # ATTEMPT:N received, waiting for START
    RUNNING = auto()          # START received, loop running (expecting HB)
    GLITCH_SUCCESS = auto()   # SUCCESS message received
    CRASHED = auto()          # No response, presumed crashed
    POWER_CYCLING = auto()    # Currently power cycling


class GlitchResult(Enum):
    """Result of a glitch attempt"""
    NOTHING = auto()       # No effect observed
    GLITCH = auto()        # Successful glitch (escaped loop)
    SKIPPED = auto()       # Skipped instructions detected (partial effect)
    CRASH = auto()         # Target crashed
    TIMEOUT = auto()       # Timeout waiting for response
    RESET = auto()         # Had to power cycle


@dataclass
class GlitchAttempt:
    """Record of a single glitch attempt"""
    attempt_number: int
    x: float
    y: float
    z: float
    delay_ns: int
    pulse_ns: int
    result: GlitchResult
    iteration_count: int = 0
    heartbeats_received: int = 0
    response_time_ms: float = 0.0
    raw_response: str = ""
    timestamp: float = field(default_factory=time.time)


@dataclass
class ScanLocation:
    """Statistics for a single scan location"""
    x: float
    y: float
    z: float
    glitch_count: int = 0
    skipped_count: int = 0  # Skipped instructions (partial glitch effect)
    crash_count: int = 0
    nothing_count: int = 0
    timeout_count: int = 0
    reset_count: int = 0
    total_attempts: int = 0
    attempts: List[GlitchAttempt] = field(default_factory=list)

    @property
    def glitch_rate(self) -> float:
        return self.glitch_count / self.total_attempts if self.total_attempts > 0 else 0.0

    @property
    def skipped_rate(self) -> float:
        return self.skipped_count / self.total_attempts if self.total_attempts > 0 else 0.0

    @property
    def crash_rate(self) -> float:
        return self.crash_count / self.total_attempts if self.total_attempts > 0 else 0.0

    @property
    def effect_rate(self) -> float:
        """Combined rate of any effect (glitch + skipped)"""
        return (self.glitch_count + self.skipped_count) / self.total_attempts if self.total_attempts > 0 else 0.0


class FaultierController:
    """
    Enhanced EMFI Controller with Faultier Integration

    Features:
    - Uses Faultier library for glitching and power cycling
    - State machine for parsing target device responses
    - Automatic crash detection and recovery
    - Detailed statistics tracking
    """

    # Timing constants
    HEARTBEAT_TIMEOUT_MS = 5000      # Max time between heartbeats before crash detection
    ATTEMPT_TIMEOUT_MS = 10000       # Max time for entire attempt
    POWER_CYCLE_WAIT_MS = 1000       # Wait after power cycle for target to boot
    READY_TIMEOUT_MS = 15000         # Max time to wait for READY after power cycle

    def __init__(self):
        # Faultier device (for power cycling and glitch timing)
        self.faultier: Optional[Faultier] = None
        self.faultier_connected = False

        # Faulty Cat device (EMP generator)
        self.faultycat_ser: Optional[serial.Serial] = None
        self.faultycat_connected = False
        self.faultycat_armed = False

        # Serial connections
        self.printer_ser: Optional[serial.Serial] = None
        self.printer_connected = False

        self.target_ser: Optional[serial.Serial] = None
        self.target_connected = False
        self.target_baudrate = 9600  # Default baud rate

        # Target state machine
        self.target_state = TargetState.UNKNOWN
        self.current_attempt_number = 0
        self.last_heartbeat_time = 0.0
        self.heartbeat_count = 0
        self.last_iteration = 0

        # Response buffer and parsing
        self.response_buffer = ""
        self.response_queue: queue.Queue = queue.Queue()
        self.reader_thread: Optional[threading.Thread] = None
        self.reader_running = False

        # Position tracking
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        # Chip area definition
        self.origin_set = False
        self.top_right_x: Optional[float] = None
        self.top_right_y: Optional[float] = None
        self.top_right_set = False

        # Glitch parameters (Faultier uses nanoseconds)
        self.glitch_delay_ns = 1000      # Delay from trigger to glitch
        self.glitch_pulse_ns = 100       # Glitch pulse width

        # Scan data
        self.scan_locations: List[ScanLocation] = []
        self.all_attempts: List[GlitchAttempt] = []
        self.total_power_cycles = 0

        # Scanning state
        self.scanning = False

        # Callbacks
        self.on_state_change: Optional[Callable[[TargetState], None]] = None
        self.on_message: Optional[Callable[[str], None]] = None

    # ======================== FAULTIER CONNECTION ========================

    def connect_faultier(self) -> tuple[bool, str]:
        """Connect to Faultier device"""
        if not FAULTIER_AVAILABLE:
            return False, "Faultier library not installed. Run: pip install faultier"

        try:
            self.faultier = Faultier()
            self.faultier.default_settings()
            self.faultier_connected = True
            return True, "Connected to Faultier successfully!"
        except Exception as e:
            self.faultier = None
            self.faultier_connected = False
            return False, f"Failed to connect to Faultier: {str(e)}"

    def disconnect_faultier(self) -> tuple[bool, str]:
        """Disconnect from Faultier device"""
        try:
            if self.faultier:
                self.faultier.default_settings()  # Reset to safe state
                self.faultier = None
            self.faultier_connected = False
            return True, "Faultier disconnected"
        except Exception as e:
            return False, f"Error disconnecting Faultier: {str(e)}"

    def configure_faultier_trigger(self,
                                    trigger_type: str = "RISING_EDGE",
                                    trigger_source: str = "EXT0",
                                    glitch_output: str = "CROWBAR") -> tuple[bool, str]:
        """
        Configure Faultier trigger settings

        Args:
            trigger_type: NONE, LOW, HIGH, RISING_EDGE, FALLING_EDGE, PULSE_POSITIVE, PULSE_NEGATIVE
            trigger_source: EXT0, EXT1
            glitch_output: CROWBAR, MUX0, MUX1, MUX2, EXT0, EXT1, NONE
        """
        if not self.faultier_connected or not self.faultier:
            return False, "Faultier not connected"

        try:
            # Map string names to Faultier constants
            trigger_type_map = {
                "NONE": Faultier.TRIGGER_NONE,
                "LOW": Faultier.TRIGGER_LOW,
                "HIGH": Faultier.TRIGGER_HIGH,
                "RISING_EDGE": Faultier.TRIGGER_RISING_EDGE,
                "FALLING_EDGE": Faultier.TRIGGER_FALLING_EDGE,
                "PULSE_POSITIVE": Faultier.TRIGGER_PULSE_POSITIVE,
                "PULSE_NEGATIVE": Faultier.TRIGGER_PULSE_NEGATIVE,
            }

            trigger_source_map = {
                "EXT0": Faultier.TRIGGER_IN_EXT0,
                "EXT1": Faultier.TRIGGER_IN_EXT1,
            }

            glitch_output_map = {
                "CROWBAR": Faultier.OUT_CROWBAR,
                "MUX0": Faultier.OUT_MUX0,
                "MUX1": Faultier.OUT_MUX1,
                "MUX2": Faultier.OUT_MUX2,
                "EXT0": Faultier.OUT_EXT0,
                "EXT1": Faultier.OUT_EXT1,
                "NONE": Faultier.OUT_NONE,
            }

            self.faultier.configure_glitcher(
                trigger_type=trigger_type_map.get(trigger_type, Faultier.TRIGGER_RISING_EDGE),
                trigger_source=trigger_source_map.get(trigger_source, Faultier.TRIGGER_IN_EXT0),
                glitch_output=glitch_output_map.get(glitch_output, Faultier.OUT_CROWBAR),
                delay=self.glitch_delay_ns,
                pulse=self.glitch_pulse_ns,
                power_cycle_length=0,
                power_cycle_output=Faultier.OUT_NONE,
                trigger_pull_configuration=Faultier.TRIGGER_PULL_NONE
            )

            return True, f"Faultier configured: trigger={trigger_type}, source={trigger_source}, output={glitch_output}"
        except Exception as e:
            return False, f"Error configuring Faultier: {str(e)}"

    def set_glitch_parameters(self, delay_ns: int, pulse_ns: int) -> tuple[bool, str]:
        """Set glitch timing parameters"""
        self.glitch_delay_ns = delay_ns
        self.glitch_pulse_ns = pulse_ns

        if self.faultier_connected and self.faultier:
            # Reconfigure with new parameters
            return self.configure_faultier_trigger()

        return True, f"Glitch parameters set: delay={delay_ns}ns, pulse={pulse_ns}ns"

    # ======================== FAULTY CAT (EMP GENERATOR) ========================

    def connect_faultycat(self, port: str, baudrate: int = 115200) -> tuple[bool, str]:
        """Connect to Faulty Cat EMP generator device"""
        try:
            self.faultycat_ser = serial.Serial(port, baudrate, timeout=1)
            time.sleep(0.5)
            self.faultycat_connected = True
            self.faultycat_armed = False

            # Disarm on connect for safety
            self._send_faultycat_command("d")

            return True, f"Connected to Faulty Cat on {port}"
        except Exception as e:
            self.faultycat_connected = False
            return False, f"Failed to connect to Faulty Cat: {str(e)}"

    def disconnect_faultycat(self) -> tuple[bool, str]:
        """Disconnect from Faulty Cat device"""
        try:
            if self.faultycat_connected:
                # Disarm before disconnecting
                self._send_faultycat_command("d")
                self.faultycat_armed = False

            if self.faultycat_ser:
                self.faultycat_ser.close()
                self.faultycat_ser = None

            self.faultycat_connected = False
            return True, "Faulty Cat disconnected"
        except Exception as e:
            return False, f"Error disconnecting Faulty Cat: {str(e)}"

    def _send_faultycat_command(self, command: str) -> Optional[str]:
        """Send command to Faulty Cat device"""
        if not self.faultycat_connected or not self.faultycat_ser:
            return None

        try:
            self.faultycat_ser.write((command + "\r\n").encode())
            time.sleep(0.05)

            response = ""
            start_time = time.time()
            while time.time() - start_time < 0.2:
                if self.faultycat_ser.in_waiting:
                    response += self.faultycat_ser.read(self.faultycat_ser.in_waiting).decode('utf-8', errors='ignore')
                    break
                time.sleep(0.02)

            return response
        except Exception as e:
            self._log_message(f"Faulty Cat command error: {e}")
            return None

    def arm_faultycat(self) -> tuple[bool, str]:
        """Arm the Faulty Cat (charges the EMP capacitor)"""
        if not self.faultycat_connected:
            return False, "Faulty Cat not connected"

        try:
            # Disarm first for safety
            self._send_faultycat_command("d")
            time.sleep(0.1)
            # Arm (charge)
            self._send_faultycat_command("a")
            time.sleep(0.1)
            self.faultycat_armed = True
            self._log_message("Faulty Cat armed (charging)")
            return True, "Faulty Cat armed and charging"
        except Exception as e:
            return False, f"Error arming Faulty Cat: {str(e)}"

    def disarm_faultycat(self) -> tuple[bool, str]:
        """Disarm the Faulty Cat"""
        if not self.faultycat_connected:
            return False, "Faulty Cat not connected"

        try:
            self._send_faultycat_command("d")
            self.faultycat_armed = False
            self._log_message("Faulty Cat disarmed")
            return True, "Faulty Cat disarmed"
        except Exception as e:
            return False, f"Error disarming Faulty Cat: {str(e)}"

    def pulse_faultycat(self) -> tuple[bool, str]:
        """
        Fire the Faulty Cat EMP pulse.
        This arms (charges) first if not already armed, then fires.
        """
        if not self.faultycat_connected:
            return False, "Faulty Cat not connected"

        try:
            # Arm if not already armed
            if not self.faultycat_armed:
                self._log_message("Arming Faulty Cat before pulse...")
                self._send_faultycat_command("d")
                time.sleep(0.05)
                self._send_faultycat_command("a")
                time.sleep(0.2)  # Wait for capacitor to charge
                self.faultycat_armed = True

            # Fire pulse
            self._send_faultycat_command("p")
            self._log_message("Faulty Cat PULSE fired!")

            # After pulse, it's disarmed
            self.faultycat_armed = False

            return True, "Faulty Cat pulse fired"
        except Exception as e:
            return False, f"Error firing Faulty Cat pulse: {str(e)}"

    def perform_emp_pulse_attempt(self, x: float, y: float, z: float,
                                   expected_heartbeats: int = 5,
                                   monitor_time_sec: float = 2.0) -> GlitchAttempt:
        """
        Perform a single EMP pulse attempt using Faulty Cat and detect the result.

        This monitors the target for:
        - SUCCESS messages (full glitch - escaped loop)
        - Skipped heartbeats (partial effect - fewer HB than expected)
        - Normal operation (all heartbeats received)
        - Crash/timeout (no response)

        Args:
            x, y, z: Current position
            expected_heartbeats: Expected number of heartbeats in monitor_time
            monitor_time_sec: How long to monitor after pulse

        Returns:
            GlitchAttempt with result
        """
        attempt = GlitchAttempt(
            attempt_number=self.current_attempt_number,
            x=x, y=y, z=z,
            delay_ns=0,
            pulse_ns=0,
            result=GlitchResult.NOTHING
        )

        start_time = time.time()
        self.heartbeat_count = 0
        heartbeats_before = 0

        # Clear buffer before pulse
        self._clear_response_buffer()

        # Count heartbeats for a moment before pulse to establish baseline
        baseline_start = time.time()
        while time.time() - baseline_start < 0.5:
            line = self._read_target_line(timeout=0.05)
            if line and "HB:" in line:
                heartbeats_before += 1
                self.last_heartbeat_time = time.time()

        # Fire the EMP pulse
        if not self.faultycat_connected:
            attempt.result = GlitchResult.NOTHING
            attempt.raw_response = "Faulty Cat not connected"
            return self._finalize_attempt(attempt, start_time)

        success, msg = self.pulse_faultycat()
        if not success:
            attempt.result = GlitchResult.NOTHING
            attempt.raw_response = f"Pulse failed: {msg}"
            return self._finalize_attempt(attempt, start_time)

        # Monitor for response after pulse
        monitor_start = time.time()
        heartbeats_after = 0
        success_detected = False
        skipped_detected = False
        last_iteration = 0
        expected_iteration = 0
        response_lines = []

        while time.time() - monitor_start < monitor_time_sec:
            line = self._read_target_line(timeout=0.05)
            if line:
                response_lines.append(line)
                self.last_heartbeat_time = time.time()

                # Check for full glitch success
                if "SUCCESS" in line:
                    success_detected = True
                    self._log_message(f"GLITCH SUCCESS: {line}")
                    break

                # Check for heartbeat and iteration count
                if line.startswith("HB:"):
                    heartbeats_after += 1
                    try:
                        current_iteration = int(line.split(":")[1])
                        # Check for skipped iterations
                        if expected_iteration > 0 and current_iteration > expected_iteration + 1:
                            skipped_detected = True
                            self._log_message(f"SKIPPED: Expected iter {expected_iteration + 1}, got {current_iteration}")
                        expected_iteration = current_iteration
                        last_iteration = current_iteration
                    except (IndexError, ValueError):
                        pass

                # Check for READY (normal cycle complete)
                if line == "READY":
                    break

        # Determine result
        attempt.heartbeats_received = heartbeats_after
        attempt.iteration_count = last_iteration
        attempt.raw_response = "\n".join(response_lines[-10:])  # Last 10 lines

        if success_detected:
            attempt.result = GlitchResult.GLITCH
            self._set_state(TargetState.GLITCH_SUCCESS)
        elif skipped_detected:
            # Only flag as SKIPPED if iteration numbers actually jumped
            # This is clear evidence of skipped loop iterations
            attempt.result = GlitchResult.SKIPPED
            self._log_message(f"Skipped instructions detected at ({x:.2f}, {y:.2f}, {z:.2f})")
        elif heartbeats_after == 0:
            # No heartbeats - likely crashed
            attempt.result = GlitchResult.CRASH
            self._set_state(TargetState.CRASHED)
            self._log_message(f"No heartbeats after pulse - target crashed at ({x:.2f}, {y:.2f}, {z:.2f})")
        else:
            # Normal operation (heartbeat count alone is not reliable for detecting skips)
            attempt.result = GlitchResult.NOTHING

        self.current_attempt_number += 1
        return self._finalize_attempt(attempt, start_time)

    # ======================== POWER CYCLING ========================

    # Power cycle output mapping
    POWER_OUTPUT_MAP = {
        "CROWBAR": 0,  # OUT_CROWBAR
        "MUX0": 1,     # OUT_MUX0
        "MUX1": 2,     # OUT_MUX1
        "MUX2": 3,     # OUT_MUX2
        "EXT0": 4,     # OUT_EXT0
        "EXT1": 5,     # OUT_EXT1
        "NONE": 6,     # OUT_NONE
    }

    def power_cycle_target(self, length_ms: int = 500) -> tuple[bool, str]:
        """
        Power cycle the target device using Faultier via subprocess.

        This uses a separate process to avoid thread/signal issues with the
        Faultier library's @no_interrupt decorator.

        Args:
            power_output: Which Faultier output controls target power
                          Options: CROWBAR, MUX0, MUX1, MUX2, EXT0, EXT1
            length_ms: How long to hold power off in milliseconds

        Returns:
            (success, message)
        """
        self._set_state(TargetState.POWER_CYCLING)
        self.total_power_cycles += 1

        self._log_message(f"Power cycling target via MUX0 for {length_ms}ms (cycle #{self.total_power_cycles})...")

        # Use subprocess to avoid thread/signal issues
        # The power_cycle_faultier.py script runs in its own process with its own main thread
        script_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "power_cycle_faultier.py")

        try:
            result = subprocess.run(
                ["python3", script_path, "MUX0", str(length_ms)],
                capture_output=True,
                text=True,
                timeout=10  # 10 second timeout
            )

            if result.returncode == 0:
                self._log_message(f"Power cycle complete: {result.stdout.strip()}")

                # Wait for target to boot
                time.sleep(self.POWER_CYCLE_WAIT_MS / 1000.0)

                # Clear any stale data in the buffer
                self._clear_response_buffer()

                # Wait for target to send READY (short timeout, don't block)
                if self._wait_for_ready(timeout_ms=3000):
                    self._set_state(TargetState.READY)
                    return True, "Power cycle complete, target is ready"
                else:
                    self._set_state(TargetState.UNKNOWN)
                    return True, "Power cycle complete (target state unknown)"
            else:
                error_msg = result.stderr.strip() or result.stdout.strip() or "Unknown error"
                self._log_message(f"Power cycle failed: {error_msg}")
                self._set_state(TargetState.CRASHED)
                return False, f"Power cycle failed: {error_msg}"

        except subprocess.TimeoutExpired:
            self._log_message("Power cycle subprocess timed out")
            self._set_state(TargetState.CRASHED)
            return False, "Power cycle timed out - Faultier may need to be unplugged/replugged"

        except FileNotFoundError:
            self._log_message(f"Power cycle script not found: {script_path}")
            self._set_state(TargetState.CRASHED)
            return False, f"Power cycle script not found: {script_path}"

        except Exception as e:
            self._log_message(f"Power cycle error: {e}")
            self._set_state(TargetState.CRASHED)
            return False, f"Power cycle error: {str(e)}"

    def _wait_for_ready(self, timeout_ms: Optional[int] = None) -> bool:
        """Wait for target to send READY message"""
        if timeout_ms is None:
            timeout_ms = self.READY_TIMEOUT_MS

        start_time = time.time()
        timeout_sec = timeout_ms / 1000.0

        while (time.time() - start_time) < timeout_sec:
            response = self._read_target_line(timeout=0.1)
            if response:
                self._log_message(f"Target: {response}")
                if "READY" in response.upper():
                    return True
                if "===" in response:  # Startup banner
                    continue
            time.sleep(0.01)

        return False

    # ======================== TARGET COMMUNICATION ========================

    def connect_target(self, port: str, baudrate: int = 115200) -> tuple[bool, str]:
        """Connect to target device"""
        try:
            self.target_ser = serial.Serial(port, baudrate, timeout=0.1)
            time.sleep(0.5)
            self.target_connected = True
            self.target_baudrate = baudrate

            # Start reader thread
            self._start_reader_thread()

            # Check for ready state
            self._clear_response_buffer()
            if self._wait_for_ready(timeout_ms=3000):
                self._set_state(TargetState.READY)
                return True, "Connected to target device (READY)"
            else:
                self._set_state(TargetState.UNKNOWN)
                return True, "Connected to target device (state unknown)"

        except Exception as e:
            return False, f"Failed to connect to target: {str(e)}"

    def connect_target_via_faultier(self, baudrate: int = 9600) -> tuple[bool, str]:
        """
        Connect to target device via Faultier's UART passthrough.

        The Faultier has a serial passthrough interface that connects to the
        target's UART TX/RX lines. This is often called "faulty cat" mode.
        """
        if not self.faultier_connected or not self.faultier:
            return False, "Faultier must be connected first"

        try:
            # Get the Faultier's serial passthrough path
            serial_path = self.faultier.get_serial_path()
            if not serial_path:
                return False, "Could not get Faultier serial path"

            self._log_message(f"Connecting to target via Faultier at {serial_path}")

            self.target_ser = serial.Serial(serial_path, baudrate, timeout=0.1)
            time.sleep(0.3)
            self.target_connected = True
            self.target_baudrate = baudrate

            # Start reader thread
            self._start_reader_thread()

            # Check for ready state (may already be running)
            self._clear_response_buffer()
            if self._wait_for_ready(timeout_ms=2000):
                self._set_state(TargetState.READY)
                return True, f"Connected to target via Faultier (READY) at {baudrate} baud"
            else:
                # Target may be in a loop already
                self._set_state(TargetState.RUNNING)
                return True, f"Connected to target via Faultier (running) at {baudrate} baud"

        except Exception as e:
            return False, f"Failed to connect to target via Faultier: {str(e)}"

    def get_faultier_serial_path(self) -> Optional[str]:
        """Get the Faultier's serial passthrough path"""
        if self.faultier_connected and self.faultier:
            try:
                return self.faultier.get_serial_path()
            except Exception:
                pass
        return None

    def disconnect_target(self) -> tuple[bool, str]:
        """Disconnect from target device"""
        try:
            self._stop_reader_thread()

            if self.target_connected and self.target_ser:
                self.target_ser.close()
            self.target_connected = False
            self._set_state(TargetState.UNKNOWN)
            return True, "Target disconnected"
        except Exception as e:
            return False, f"Error disconnecting target: {str(e)}"

    def _start_reader_thread(self):
        """Start background thread for reading target responses"""
        if self.reader_running:
            return

        self.reader_running = True
        self.reader_thread = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader_thread.start()

    def _stop_reader_thread(self):
        """Stop the reader thread"""
        self.reader_running = False
        if self.reader_thread:
            self.reader_thread.join(timeout=1.0)
            self.reader_thread = None

    def _reader_loop(self):
        """Background loop for reading target serial data"""
        while self.reader_running and self.target_connected:
            try:
                if self.target_ser and self.target_ser.in_waiting:
                    data = self.target_ser.read(self.target_ser.in_waiting).decode('utf-8', errors='ignore')
                    self.response_buffer += data

                    # Process complete lines
                    while '\n' in self.response_buffer:
                        line, self.response_buffer = self.response_buffer.split('\n', 1)
                        line = line.strip('\r')
                        if line:
                            self.response_queue.put(line)

            except Exception as e:
                if self.reader_running:
                    print(f"Reader error: {e}")

            time.sleep(0.001)

    def _read_target_line(self, timeout: float = 0.1) -> Optional[str]:
        """Read a line from the target response queue"""
        try:
            return self.response_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _clear_response_buffer(self):
        """Clear the response queue"""
        while not self.response_queue.empty():
            try:
                self.response_queue.get_nowait()
            except queue.Empty:
                break
        self.response_buffer = ""

    def _set_state(self, new_state: TargetState):
        """Update target state and notify callback"""
        old_state = self.target_state
        self.target_state = new_state

        if old_state != new_state and self.on_state_change:
            self.on_state_change(new_state)

    def _log_message(self, message: str):
        """Log a message via callback"""
        if self.on_message:
            self.on_message(message)
        else:
            print(message)

    # ======================== GLITCH DETECTION ========================

    def perform_glitch_attempt(self, x: float, y: float, z: float) -> GlitchAttempt:
        """
        Perform a single glitch attempt and detect the result

        This implements the full state machine:
        1. Wait for ATTEMPT:N message
        2. Wait for START message
        3. Monitor heartbeats during loop
        4. Detect SUCCESS or timeout/crash
        5. Power cycle if needed

        Returns:
            GlitchAttempt with result and statistics
        """
        attempt = GlitchAttempt(
            attempt_number=self.current_attempt_number,
            x=x, y=y, z=z,
            delay_ns=self.glitch_delay_ns,
            pulse_ns=self.glitch_pulse_ns,
            result=GlitchResult.NOTHING
        )

        start_time = time.time()
        self.heartbeat_count = 0
        self.last_heartbeat_time = start_time

        # Clear buffer before attempt
        self._clear_response_buffer()

        try:
            # Wait for ATTEMPT message (target sets trigger high)
            if not self._wait_for_attempt(timeout_ms=2000):
                self._log_message("No ATTEMPT received, checking target...")
                if self._check_and_recover():
                    attempt.result = GlitchResult.RESET
                else:
                    attempt.result = GlitchResult.CRASH
                return self._finalize_attempt(attempt, start_time)

            self._set_state(TargetState.ATTEMPT_STARTED)

            # Wait for START message (trigger is now high, glitcher should be armed)
            if not self._wait_for_start(timeout_ms=1000):
                self._log_message("No START received")
                attempt.result = GlitchResult.CRASH
                if self._check_and_recover():
                    attempt.result = GlitchResult.RESET
                return self._finalize_attempt(attempt, start_time)

            self._set_state(TargetState.RUNNING)

            # Arm the Faultier to glitch on trigger (trigger is already high)
            if self.faultier_connected and self.faultier:
                # The target has set trigger HIGH, now we arm and it will glitch
                self.faultier.glitch(delay=self.glitch_delay_ns, pulse=self.glitch_pulse_ns)

            # Monitor for result
            result = self._monitor_for_result(timeout_ms=self.ATTEMPT_TIMEOUT_MS)
            attempt.result = result
            attempt.heartbeats_received = self.heartbeat_count
            attempt.iteration_count = self.last_iteration

            # Handle crash recovery
            if result in (GlitchResult.CRASH, GlitchResult.TIMEOUT):
                self._log_message(f"Target appears crashed/timed out after {self.heartbeat_count} heartbeats")
                if self._check_and_recover():
                    attempt.result = GlitchResult.RESET

        except Exception as e:
            self._log_message(f"Glitch attempt error: {e}")
            attempt.result = GlitchResult.CRASH
            self._check_and_recover()

        return self._finalize_attempt(attempt, start_time)

    def _wait_for_attempt(self, timeout_ms: int) -> bool:
        """Wait for ATTEMPT:N message from target"""
        start_time = time.time()
        timeout_sec = timeout_ms / 1000.0

        while (time.time() - start_time) < timeout_sec:
            line = self._read_target_line(timeout=0.05)
            if line:
                self._log_message(f"Target: {line}")
                if line.startswith("ATTEMPT:"):
                    try:
                        self.current_attempt_number = int(line.split(":")[1])
                        return True
                    except (IndexError, ValueError):
                        pass
        return False

    def _wait_for_start(self, timeout_ms: int) -> bool:
        """Wait for START message from target"""
        start_time = time.time()
        timeout_sec = timeout_ms / 1000.0

        while (time.time() - start_time) < timeout_sec:
            line = self._read_target_line(timeout=0.05)
            if line:
                self._log_message(f"Target: {line}")
                if line == "START":
                    return True
        return False

    def _monitor_for_result(self, timeout_ms: int) -> GlitchResult:
        """
        Monitor target for glitch result

        Watches for:
        - SUCCESS message (glitch worked!)
        - READY message (normal cycle complete, no effect)
        - Heartbeat timeout (crash)
        - Overall timeout
        """
        start_time = time.time()
        timeout_sec = timeout_ms / 1000.0
        self.last_heartbeat_time = start_time

        while (time.time() - start_time) < timeout_sec:
            line = self._read_target_line(timeout=0.05)

            if line:
                self._log_message(f"Target: {line}")

                # Check for success
                if line.startswith("SUCCESS:"):
                    self._parse_success(line)
                    self._set_state(TargetState.GLITCH_SUCCESS)
                    return GlitchResult.GLITCH

                # Check for ready (normal completion, no glitch effect)
                if line == "READY":
                    self._set_state(TargetState.READY)
                    return GlitchResult.NOTHING

                # Check for heartbeat
                if line.startswith("HB:"):
                    self.heartbeat_count += 1
                    self.last_heartbeat_time = time.time()
                    try:
                        self.last_iteration = int(line.split(":")[1])
                    except (IndexError, ValueError):
                        pass

            # Check heartbeat timeout
            if self.target_state == TargetState.RUNNING:
                time_since_heartbeat = (time.time() - self.last_heartbeat_time) * 1000
                if time_since_heartbeat > self.HEARTBEAT_TIMEOUT_MS:
                    self._log_message(f"Heartbeat timeout ({time_since_heartbeat:.0f}ms)")
                    self._set_state(TargetState.CRASHED)
                    return GlitchResult.CRASH

        # Overall timeout
        self._set_state(TargetState.CRASHED)
        return GlitchResult.TIMEOUT

    def _parse_success(self, line: str):
        """Parse SUCCESS message for details"""
        # Format: SUCCESS:ESCAPED_LOOP:attempt_num:ITER:iteration_count
        try:
            parts = line.split(":")
            if len(parts) >= 5:
                self.last_iteration = int(parts[4])
        except (IndexError, ValueError):
            pass

    def _check_and_recover(self) -> bool:
        """Check if target needs recovery and attempt power cycle via subprocess"""
        self._log_message("Attempting recovery via power cycle...")
        success, msg = self.power_cycle_target()
        self._log_message(msg)
        return success

    def _finalize_attempt(self, attempt: GlitchAttempt, start_time: float) -> GlitchAttempt:
        """Finalize attempt with timing information"""
        attempt.response_time_ms = (time.time() - start_time) * 1000
        self.all_attempts.append(attempt)
        return attempt

    # ======================== PRINTER CONTROL ========================

    def connect_printer(self, port: str, baudrate: int = 115200) -> tuple[bool, str]:
        """Connect to 3D printer for positioning"""
        try:
            self.printer_ser = serial.Serial(port, baudrate, timeout=5)
            time.sleep(2)
            self.printer_connected = True
            self._initialize_printer()
            return True, "Connected to printer successfully!"
        except Exception as e:
            return False, f"Failed to connect to printer: {str(e)}"

    def disconnect_printer(self) -> tuple[bool, str]:
        """Disconnect from 3D printer"""
        try:
            if self.printer_connected and self.printer_ser:
                self._send_gcode("M84")  # Disable motors
                self.printer_ser.close()
            self.printer_connected = False
            return True, "Printer disconnected"
        except Exception as e:
            return False, f"Error disconnecting printer: {str(e)}"

    def _initialize_printer(self):
        """Initialize printer settings"""
        self._send_gcode("M84 X Y Z S12000")
        self._send_gcode("G21")   # Millimeters
        self._send_gcode("G90")   # Absolute positioning
        self._send_gcode("M999")  # Ignore errors

    def _send_gcode(self, command: str) -> Optional[str]:
        """Send G-code command to printer"""
        if not self.printer_connected or not self.printer_ser:
            return None

        try:
            self.printer_ser.write((command + "\r\n").encode())
            time.sleep(0.05)

            response = ""
            start_time = time.time()
            while time.time() - start_time < 0.5:
                if self.printer_ser.in_waiting:
                    response += self.printer_ser.read(self.printer_ser.in_waiting).decode('utf-8', errors='ignore')
                    break
                time.sleep(0.01)

            return response
        except Exception as e:
            print(f"Error sending G-code: {e}")
            return None

    def move_to_position(self, x: float, y: float, z: float) -> tuple[bool, str]:
        """Move to absolute position"""
        if not self.printer_connected:
            return False, "Printer not connected"

        try:
            self._send_gcode(f"G1 X{x:.3f} Y{y:.3f} Z{z:.3f} F3000")
            self._send_gcode("M400")

            self.current_x = x
            self.current_y = y
            self.current_z = z

            return True, f"Moved to ({x:.2f}, {y:.2f}, {z:.2f})"
        except Exception as e:
            return False, f"Movement error: {str(e)}"

    def move_relative(self, axis: str, distance: float) -> tuple[bool, str]:
        """Move printer relatively on specified axis"""
        if not self.printer_connected:
            return False, "Printer not connected"

        try:
            self._send_gcode("G91")  # Relative positioning
            self._send_gcode(f"G1 {axis}{distance} F3000")
            self._send_gcode("M400")
            self._send_gcode("G90")  # Absolute positioning

            if axis == 'X':
                self.current_x += distance
            elif axis == 'Y':
                self.current_y += distance
            elif axis == 'Z':
                self.current_z += distance

            return True, f"Moved {axis} by {distance}mm"
        except Exception as e:
            return False, f"Movement error: {str(e)}"

    def home_all_axes(self) -> tuple[bool, str]:
        """Home all axes"""
        if not self.printer_connected:
            return False, "Printer not connected"

        try:
            self._send_gcode("G28")
            self._send_gcode("M400")
            self.current_x = 0
            self.current_y = 0
            self.current_z = 0
            self._send_gcode("M999")
            return True, "Homing complete"
        except Exception as e:
            return False, f"Homing error: {str(e)}"

    def set_origin(self) -> tuple[bool, str]:
        """Set current position as origin"""
        if not self.printer_connected:
            return False, "Printer not connected"

        try:
            self._send_gcode("G92 X0 Y0 Z0")
            self.current_x = 0
            self.current_y = 0
            self.current_z = 0
            self.origin_set = True
            return True, "Origin set at current position"
        except Exception as e:
            return False, f"Error setting origin: {str(e)}"

    def set_top_right_corner(self) -> tuple[bool, str]:
        """Mark current position as top-right corner"""
        if not self.printer_connected:
            return False, "Printer not connected"

        if not self.origin_set:
            return False, "Origin must be set first"

        if self.current_x <= 0 or self.current_y <= 0:
            return False, f"Invalid position: X={self.current_x:.2f}, Y={self.current_y:.2f}"

        self.top_right_x = self.current_x
        self.top_right_y = self.current_y
        self.top_right_set = True

        return True, f"Top-right corner set at ({self.top_right_x:.2f}, {self.top_right_y:.2f})"

    # ======================== SCANNING ========================

    def calculate_scan_grid(self, step_size: float, z_increment: float, max_z_height: float) -> Optional[Dict[str, int]]:
        """Calculate scan grid dimensions"""
        if not self.top_right_set or self.top_right_x is None or self.top_right_y is None:
            return None

        x_steps = int(self.top_right_x / step_size) + 1
        y_steps = int(self.top_right_y / step_size) + 1
        z_steps = int(max_z_height / z_increment) + 1
        total_points = x_steps * y_steps * z_steps

        return {
            'x_steps': x_steps,
            'y_steps': y_steps,
            'z_steps': z_steps,
            'total_points': total_points
        }

    def run_scan(self,
                 step_size: float,
                 z_increment: float,
                 max_z_height: float,
                 pulses_per_location: int,
                 progress_callback: Optional[Callable] = None) -> tuple[bool, str]:
        """
        Run EMFI scan over the defined area with automatic recovery
        """
        if not self.printer_connected:
            return False, "Printer not connected"

        if not self.origin_set or not self.top_right_set:
            return False, "Chip area not configured"

        grid = self.calculate_scan_grid(step_size, z_increment, max_z_height)
        if not grid:
            return False, "Failed to calculate grid"

        self.scanning = True
        self.scan_locations = []

        x_steps = grid['x_steps']
        y_steps = grid['y_steps']
        z_steps = grid['z_steps']

        location_index = 0
        total_locations = grid['total_points']

        try:
            for z_idx in range(z_steps):
                if not self.scanning:
                    break

                z_pos = z_idx * z_increment

                for y_idx in range(y_steps):
                    if not self.scanning:
                        break

                    y_pos = y_idx * step_size

                    # Snake pattern
                    if y_idx % 2 == 0:
                        x_range = range(x_steps)
                    else:
                        x_range = range(x_steps - 1, -1, -1)

                    for x_idx in x_range:
                        if not self.scanning:
                            break

                        x_pos = x_idx * step_size

                        # Move to position
                        success, msg = self.move_to_position(x_pos, y_pos, z_pos)
                        if not success:
                            return False, msg

                        time.sleep(0.05)

                        # Create location record
                        location = ScanLocation(x=x_pos, y=y_pos, z=z_pos)

                        # Perform multiple glitch attempts at this location
                        for pulse in range(pulses_per_location):
                            if not self.scanning:
                                break

                            attempt = self.perform_glitch_attempt(x_pos, y_pos, z_pos)
                            location.attempts.append(attempt)
                            location.total_attempts += 1

                            if attempt.result == GlitchResult.GLITCH:
                                location.glitch_count += 1
                            elif attempt.result == GlitchResult.SKIPPED:
                                location.skipped_count += 1
                            elif attempt.result == GlitchResult.CRASH:
                                location.crash_count += 1
                            elif attempt.result == GlitchResult.TIMEOUT:
                                location.timeout_count += 1
                            elif attempt.result == GlitchResult.RESET:
                                location.reset_count += 1
                            else:
                                location.nothing_count += 1

                            time.sleep(0.02)

                        self.scan_locations.append(location)
                        location_index += 1

                        if progress_callback:
                            progress_callback(location_index, total_locations, x_pos, y_pos, z_pos, location)

            # Move to safe position
            self._send_gcode("G1 Z30 F1000")

            return True, "Scan completed successfully"

        except Exception as e:
            return False, f"Scan error: {str(e)}"
        finally:
            self.scanning = False

    def run_emp_scan(self,
                      step_size: float,
                      z_increment: float,
                      max_z_height: float,
                      pulses_per_location: int,
                      delay_between_pulses_ms: int = 300,
                      delay_after_move_ms: int = 100,
                      delay_after_power_cycle_ms: int = 1000,
                      power_cycle_duration_ms: int = 500,
                      progress_callback: Optional[Callable] = None) -> tuple[bool, str]:
        """
        Run EMP scan over the defined area using Faulty Cat pulses.

        This scan:
        - Moves to each grid position using the 3D printer
        - Fires Faulty Cat EMP pulses at each location
        - Monitors target for SUCCESS, skipped instructions, or crash
        - Auto power cycles via Faultier MUX0 on crash detection
        - Records results for heatmap visualization

        Args:
            step_size: XY step size in mm
            z_increment: Z step size in mm
            max_z_height: Maximum Z height to scan
            pulses_per_location: Number of EMP pulses per location
            delay_between_pulses_ms: Delay between EMP pulses in milliseconds
            delay_after_move_ms: Settle time after movement in milliseconds
            delay_after_power_cycle_ms: Wait time after power cycle in milliseconds
            power_cycle_duration_ms: Power cycle pulse length in milliseconds
            progress_callback: Called after each location with progress info

        Returns:
            (success, message)
        """
        if not self.printer_connected:
            return False, "Printer not connected"

        if not self.faultycat_connected:
            return False, "Faulty Cat not connected"

        if not self.origin_set or not self.top_right_set:
            return False, "Chip area not configured"

        grid = self.calculate_scan_grid(step_size, z_increment, max_z_height)
        if not grid:
            return False, "Failed to calculate grid"

        self.scanning = True
        self.scan_locations = []

        x_steps = grid['x_steps']
        y_steps = grid['y_steps']
        z_steps = grid['z_steps']

        location_index = 0
        total_locations = grid['total_points']
        consecutive_crashes = 0
        max_consecutive_crashes = 5  # Stop if too many crashes in a row

        self._log_message(f"Starting EMP scan: {x_steps}x{y_steps}x{z_steps} grid, {pulses_per_location} pulses/location")

        try:
            for z_idx in range(z_steps):
                if not self.scanning:
                    break

                z_pos = z_idx * z_increment

                for y_idx in range(y_steps):
                    if not self.scanning:
                        break

                    y_pos = y_idx * step_size

                    # Snake pattern for efficient movement
                    if y_idx % 2 == 0:
                        x_range = range(x_steps)
                    else:
                        x_range = range(x_steps - 1, -1, -1)

                    for x_idx in x_range:
                        if not self.scanning:
                            break

                        x_pos = x_idx * step_size

                        # Move to position
                        success, msg = self.move_to_position(x_pos, y_pos, z_pos)
                        if not success:
                            self._log_message(f"Movement failed: {msg}")
                            continue

                        time.sleep(delay_after_move_ms / 1000.0)  # Settle time

                        # Create location record
                        location = ScanLocation(x=x_pos, y=y_pos, z=z_pos)

                        # Perform multiple EMP pulses at this location
                        for pulse_num in range(pulses_per_location):
                            if not self.scanning:
                                break

                            # Perform EMP pulse and monitor result
                            attempt = self.perform_emp_pulse_attempt(x_pos, y_pos, z_pos)
                            location.attempts.append(attempt)
                            location.total_attempts += 1

                            # Record result
                            if attempt.result == GlitchResult.GLITCH:
                                location.glitch_count += 1
                                consecutive_crashes = 0
                                self._log_message(f"*** GLITCH at ({x_pos:.2f}, {y_pos:.2f}, {z_pos:.2f})! ***")
                            elif attempt.result == GlitchResult.SKIPPED:
                                location.skipped_count += 1
                                consecutive_crashes = 0
                                self._log_message(f"Skipped at ({x_pos:.2f}, {y_pos:.2f}, {z_pos:.2f})")
                            elif attempt.result == GlitchResult.CRASH:
                                location.crash_count += 1
                                consecutive_crashes += 1

                                # Auto power cycle on crash
                                self._log_message(f"Crash at ({x_pos:.2f}, {y_pos:.2f}, {z_pos:.2f}) - power cycling...")
                                pc_success, pc_msg = self.power_cycle_target(length_ms=power_cycle_duration_ms)
                                if pc_success:
                                    self._log_message("Power cycle complete, continuing scan")
                                    consecutive_crashes = 0
                                    time.sleep(delay_after_power_cycle_ms / 1000.0)  # Wait for target to stabilize
                                else:
                                    self._log_message(f"Power cycle failed: {pc_msg}")

                                if consecutive_crashes >= max_consecutive_crashes:
                                    self._log_message(f"Too many consecutive crashes ({consecutive_crashes}), stopping scan")
                                    self.scanning = False
                                    break

                            elif attempt.result == GlitchResult.TIMEOUT:
                                location.timeout_count += 1
                                consecutive_crashes += 1

                                # Power cycle on timeout too
                                self._log_message(f"Timeout at ({x_pos:.2f}, {y_pos:.2f}, {z_pos:.2f}) - power cycling...")
                                self.power_cycle_target(length_ms=power_cycle_duration_ms)
                                time.sleep(delay_after_power_cycle_ms / 1000.0)

                            elif attempt.result == GlitchResult.RESET:
                                location.reset_count += 1
                            else:
                                location.nothing_count += 1
                                consecutive_crashes = 0

                            # Delay between pulses (capacitor recharge)
                            time.sleep(delay_between_pulses_ms / 1000.0)

                        self.scan_locations.append(location)
                        location_index += 1

                        # Progress callback
                        if progress_callback:
                            progress_callback(location_index, total_locations, x_pos, y_pos, z_pos, location)

            # Move to safe position
            self._send_gcode("G1 Z30 F1000")

            return True, "EMP scan completed successfully"

        except Exception as e:
            return False, f"EMP scan error: {str(e)}"
        finally:
            self.scanning = False

    def stop_scan(self):
        """Stop the current scan"""
        self.scanning = False

        if self.printer_connected:
            self._send_gcode("G1 Z30 F1000")

    # ======================== DATA ANALYSIS ========================

    def find_optimal_glitch_location(self) -> Optional[Dict[str, Any]]:
        """Find the optimal glitch location based on scoring"""
        if not self.scan_locations:
            return None

        best_location = None
        best_score = -999999

        for loc in self.scan_locations:
            if loc.total_attempts == 0:
                continue

            # Calculate score - glitch is best, skipped is also good
            glitch_rate = loc.glitch_rate
            skipped_rate = loc.skipped_rate
            effect_rate = loc.effect_rate
            crash_rate = loc.crash_rate

            # Scoring: glitch is worth more than skipped, crashes are bad
            score = (glitch_rate * 10.0 +      # Full glitch (escaped loop) is best
                    skipped_rate * 5.0 -        # Skipped instructions is partial success
                    crash_rate * 3.0 -          # Crashes are bad
                    (loc.nothing_count / loc.total_attempts) * 1.0)

            # Bonus for high absolute glitch count
            if loc.glitch_count >= 3:
                score += 2.0

            # Bonus for any effect (including skipped)
            if loc.glitch_count + loc.skipped_count >= 2:
                score += 1.0

            # Consider neighbors
            neighbor_effect_rates = []
            for neighbor in self.scan_locations:
                if neighbor == loc:
                    continue
                dist = np.sqrt((neighbor.x - loc.x)**2 +
                             (neighbor.y - loc.y)**2 +
                             (neighbor.z - loc.z)**2)

                if dist <= 2.0:
                    neighbor_effect_rates.append(neighbor.effect_rate)

            if neighbor_effect_rates:
                avg_neighbor_rate = np.mean(neighbor_effect_rates)
                if avg_neighbor_rate > 0.2:
                    score += avg_neighbor_rate * 2.0

            if score > best_score:
                best_score = score
                best_location = {
                    'x': loc.x,
                    'y': loc.y,
                    'z': loc.z,
                    'glitch': loc.glitch_count,
                    'skipped': loc.skipped_count,
                    'crash': loc.crash_count,
                    'nothing': loc.nothing_count,
                    'total': loc.total_attempts,
                    'score': score,
                    'glitch_rate': glitch_rate,
                    'skipped_rate': skipped_rate,
                    'effect_rate': effect_rate,
                    'crash_rate': crash_rate
                }

        return best_location

    def get_statistics(self) -> Dict[str, Any]:
        """Get overall scan statistics"""
        total_glitches = sum(loc.glitch_count for loc in self.scan_locations)
        total_skipped = sum(loc.skipped_count for loc in self.scan_locations)
        total_crashes = sum(loc.crash_count for loc in self.scan_locations)
        total_timeouts = sum(loc.timeout_count for loc in self.scan_locations)
        total_resets = sum(loc.reset_count for loc in self.scan_locations)
        total_nothing = sum(loc.nothing_count for loc in self.scan_locations)
        total_attempts = sum(loc.total_attempts for loc in self.scan_locations)

        return {
            'total_attempts': total_attempts,
            'total_glitches': total_glitches,
            'total_skipped': total_skipped,
            'total_crashes': total_crashes,
            'total_timeouts': total_timeouts,
            'total_resets': total_resets,
            'total_nothing': total_nothing,
            'total_power_cycles': self.total_power_cycles,
            'glitch_rate': total_glitches / total_attempts if total_attempts > 0 else 0,
            'skipped_rate': total_skipped / total_attempts if total_attempts > 0 else 0,
            'effect_rate': (total_glitches + total_skipped) / total_attempts if total_attempts > 0 else 0,
            'crash_rate': total_crashes / total_attempts if total_attempts > 0 else 0,
            'locations_scanned': len(self.scan_locations)
        }

    def reset_data(self):
        """Reset all scan data"""
        self.scan_locations = []
        self.all_attempts = []
        self.total_power_cycles = 0

    # ======================== CLEANUP ========================

    def cleanup(self):
        """Cleanup all connections"""
        self.stop_scan()
        self.disconnect_target()
        self.disconnect_printer()
        self.disconnect_faultycat()
        self.disconnect_faultier()


# Compatibility wrapper for existing code
class EMFIController(FaultierController):
    """
    Compatibility wrapper for existing EMFIController interface

    This maintains backward compatibility while using the new FaultierController
    """

    def __init__(self):
        super().__init__()
        # Legacy attributes
        self.emfi_ser = None
        self.emfi_connected = False
        self.glitch_data = []
        self.total_glitches = 0
        self.total_crashes = 0
        self.total_attempts = 0

    def connect_emfi(self, port: str, baudrate: int = 115200) -> tuple[bool, str]:
        """Legacy EMFI connection - tries Faultier first, falls back to serial"""
        # Try Faultier library first
        success, msg = self.connect_faultier()
        if success:
            self.emfi_connected = True
            return success, msg

        # Fall back to direct serial connection
        try:
            self.emfi_ser = serial.Serial(port, baudrate, timeout=2)
            time.sleep(0.5)
            self.emfi_connected = True
            return True, "Connected to EMFI device via serial"
        except Exception as e:
            return False, f"Failed to connect to EMFI device: {str(e)}"

    def disconnect_emfi(self) -> tuple[bool, str]:
        """Legacy EMFI disconnection"""
        if self.faultier_connected:
            return self.disconnect_faultier()

        try:
            if self.emfi_connected and self.emfi_ser:
                self.emfi_ser.close()
            self.emfi_connected = False
            return True, "EMFI device disconnected"
        except Exception as e:
            return False, f"Error disconnecting EMFI device: {str(e)}"

    def arm_faultycat(self) -> tuple[bool, str]:
        """Legacy arm command"""
        if self.faultier_connected:
            return self.configure_faultier_trigger()

        if not self.emfi_connected:
            return False, "EMFI device not connected"

        try:
            self.send_emfi_command("d")
            time.sleep(0.1)
            self.send_emfi_command("a")
            time.sleep(0.1)
            return True, "FaultyCat armed"
        except Exception as e:
            return False, f"Error arming: {str(e)}"

    def disarm_faultycat(self) -> tuple[bool, str]:
        """Legacy disarm command"""
        if self.faultier_connected and self.faultier:
            self.faultier.default_settings()
            return True, "Faultier disarmed"

        if not self.emfi_connected:
            return False, "EMFI device not connected"

        try:
            self.send_emfi_command("d")
            return True, "FaultyCat disarmed"
        except Exception as e:
            return False, f"Error disarming: {str(e)}"

    def send_emfi_command(self, command: str) -> Optional[str]:
        """Legacy serial command to EMFI"""
        if self.faultier_connected:
            # Faultier doesn't use serial commands
            return None

        if not self.emfi_connected or not self.emfi_ser:
            return None

        try:
            self.emfi_ser.write((command + "\r\n").encode())
            time.sleep(0.05)

            response = ""
            start_time = time.time()
            while time.time() - start_time < 0.2:
                if self.emfi_ser.in_waiting:
                    response += self.emfi_ser.read(self.emfi_ser.in_waiting).decode('utf-8', errors='ignore')
                    break
                time.sleep(0.05)

            return response
        except Exception as e:
            print(f"Error sending EMFI command: {e}")
            return None

    def trigger_emfi(self) -> str:
        """
        Legacy trigger - now with proper detection
        Returns: 'glitch', 'crash', or 'nothing'
        """
        if self.faultier_connected:
            attempt = self.perform_glitch_attempt(self.current_x, self.current_y, self.current_z)

            # Update legacy counters
            self.total_attempts += 1
            if attempt.result == GlitchResult.GLITCH:
                self.total_glitches += 1
                return 'glitch'
            elif attempt.result in (GlitchResult.CRASH, GlitchResult.TIMEOUT, GlitchResult.RESET):
                self.total_crashes += 1
                return 'crash'
            else:
                return 'nothing'

        # Legacy serial-based trigger
        if self.emfi_connected and self.emfi_ser:
            try:
                self.send_emfi_command("p")
                time.sleep(0.05)
            except Exception as e:
                print(f"Error triggering pulse: {e}")
                return 'nothing'

        # Check target response
        if self.target_connected:
            try:
                response = self.read_target_response(timeout=0.1)

                if response is None:
                    return 'crash'
                elif "SUCCESS" in response.upper() or "ESCAPED" in response.upper():
                    return 'glitch'
                else:
                    return 'nothing'
            except Exception:
                return 'crash'

        # Simulation fallback
        result_val = np.random.random()
        if result_val < 0.3:
            return 'glitch'
        elif result_val < 0.4:
            return 'crash'
        else:
            return 'nothing'

    def read_target_response(self, timeout: float = 0.5) -> Optional[str]:
        """Legacy target read"""
        if not self.target_connected or not self.target_ser:
            return None

        try:
            response = ""
            start_time = time.time()
            while time.time() - start_time < timeout:
                if self.target_ser.in_waiting:
                    response += self.target_ser.read(self.target_ser.in_waiting).decode('utf-8', errors='ignore')
                time.sleep(0.01)
            return response if response else None
        except Exception as e:
            print(f"Error reading target: {e}")
            return None

    def send_target_command(self, command: str) -> Optional[str]:
        """Legacy target command"""
        if not self.target_connected or not self.target_ser:
            return None

        try:
            self.target_ser.write((command + "\r\n").encode())
            time.sleep(0.01)
            return self.read_target_response(timeout=0.2)
        except Exception as e:
            print(f"Error sending target command: {e}")
            return None
