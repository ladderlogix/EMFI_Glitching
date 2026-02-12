"""
Enhanced EMFI Scanner GUI with Faultier Integration

Features:
- Faultier device connection with proper API
- Target state monitoring with state machine
- Automatic crash detection and power cycling
- Configurable glitch parameters (delay, pulse width)
- Trigger configuration
- Enhanced statistics with recovery counts
"""

import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure
from mpl_toolkits.mplot3d import Axes3D
import time
import glob
import os
from datetime import datetime

from faultier_controller import FaultierController, TargetState, GlitchResult

# Max serial log file size (10MB)
MAX_LOG_FILE_SIZE = 10 * 1024 * 1024


def get_available_serial_ports():
    """Scan for available serial ports on the system."""
    patterns = [
        '/dev/ttyACM*',   # Arduino, 3D printers (most common)
        '/dev/ttyUSB*',   # USB-Serial adapters
        '/dev/ttyAMA*',   # Raspberry Pi serial
        '/dev/ttyS*',     # Standard serial ports
    ]
    ports = []
    for pattern in patterns:
        ports.extend(glob.glob(pattern))
    return sorted(ports)


class EMFIFaultierGUI:
    """Enhanced GUI for EMFI Scanner with Faultier Integration"""

    def __init__(self, root):
        self.root = root
        self.root.title("EMFI Scanner Control - Faultier Edition")
        self.root.geometry("1700x950")

        # Create controller instance
        self.controller = FaultierController()

        # Set up controller callbacks
        self.controller.on_state_change = self.on_target_state_change
        self.controller.on_message = self.on_controller_message

        # Configuration parameters
        self.probe_diameter = tk.DoubleVar(value=2.0)
        self.step_size = tk.DoubleVar(value=1.0)
        self.z_increment = tk.DoubleVar(value=0.1)
        self.max_z_height = tk.DoubleVar(value=0.5)
        self.pulses_per_location = tk.IntVar(value=10)

        # Serial port parameters
        self.printer_port = tk.StringVar(value="/dev/ttyACM0")
        self.printer_baudrate = tk.IntVar(value=250000)
        self.faultycat_port = tk.StringVar(value="/dev/ttyACM1")
        self.faultycat_baudrate = tk.IntVar(value=115200)
        self.target_port = tk.StringVar(value="/dev/ttyACM2")
        self.target_baudrate = tk.IntVar(value=9600)

        # Movement step size
        self.move_step = tk.DoubleVar(value=1.0)

        # Faulty Cat pulse count
        self.pulse_count = tk.IntVar(value=1)

        # Timing delays (in milliseconds)
        self.delay_between_pulses = tk.IntVar(value=300)      # Delay between EMP pulses (ms)
        self.delay_after_move = tk.IntVar(value=100)          # Settle time after movement (ms)
        self.delay_after_power_cycle = tk.IntVar(value=1000)  # Wait after power cycle (ms)
        self.power_cycle_duration = tk.IntVar(value=500)      # Power cycle pulse length (ms)

        # Serial logging
        self.serial_log_file = None
        self.serial_log_path = None
        self.serial_log_size = 0
        self.serial_log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), "serial_logs")
        self.serial_logging_enabled = tk.BooleanVar(value=True)

        # Thread management
        self.scan_thread = None
        self.monitor_thread = None
        self.monitor_running = False

        # Board status monitoring
        self.board_monitor_thread = None
        self.board_monitor_running = False
        self.last_heartbeat_time = 0.0
        self.board_alive = False
        self.heartbeat_timeout_sec = 2.0  # Board considered dead if no heartbeat in 2 seconds

        # Flag to stop periodic updates on close
        self.closing = False

        # Build GUI
        self.create_widgets()

        # Start periodic UI updates
        self.update_ui_periodically()

    def create_widgets(self):
        # Main container with two panes
        main_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Left panel - Controls (split into two columns)
        left_container = ttk.Frame(main_pane)
        main_pane.add(left_container, width=900)

        # Create two columns in left container
        left_column = ttk.Frame(left_container)
        left_column.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        right_column = ttk.Frame(left_container)
        right_column.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))

        # Right panel - Visualization
        right_frame = ttk.Frame(main_pane)
        main_pane.add(right_frame)

        # Build left column sections
        self.create_connection_section(left_column)
        self.create_faultier_config_section(left_column)
        self.create_manual_control_section(left_column)

        # Build right column sections
        self.create_target_status_section(right_column)
        self.create_scan_config_section(right_column)
        self.create_scan_control_section(right_column)
        self.create_status_section(right_column)

        # Build right panel - visualization
        self.create_visualization_section(right_frame)

    def create_connection_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Device Connections", padding=8)
        frame.pack(fill=tk.X, padx=5, pady=5)

        # Refresh button for all serial ports
        refresh_frame = ttk.Frame(frame)
        refresh_frame.pack(fill=tk.X, pady=(0, 5))
        ttk.Button(refresh_frame, text="↻ Refresh Ports", command=self.refresh_serial_ports).pack(side=tk.RIGHT)

        # Printer Connection
        printer_frame = ttk.LabelFrame(frame, text="3D Printer (Required)", padding=5)
        printer_frame.pack(fill=tk.X, pady=3)

        ttk.Label(printer_frame, text="Port:", font=("Arial", 8)).grid(row=0, column=0, sticky=tk.W, pady=1)
        self.printer_port_combo = ttk.Combobox(printer_frame, textvariable=self.printer_port, width=15, font=("Arial", 8))
        self.printer_port_combo.grid(row=0, column=1, pady=1)

        ttk.Label(printer_frame, text="Baud:", font=("Arial", 8)).grid(row=1, column=0, sticky=tk.W, pady=1)
        self.printer_baud_combo = ttk.Combobox(printer_frame, textvariable=self.printer_baudrate, width=15, font=("Arial", 8),
                                               values=[250000, 115200, 57600, 38400, 19200, 9600])
        self.printer_baud_combo.grid(row=1, column=1, pady=1)

        self.printer_connect_btn = ttk.Button(printer_frame, text="Connect", command=self.connect_printer)
        self.printer_connect_btn.grid(row=2, column=0, columnspan=2, pady=3)

        self.printer_status = ttk.Label(printer_frame, text="Disconnected", foreground="red", font=("Arial", 8))
        self.printer_status.grid(row=3, column=0, columnspan=2)

        # Faultier Power Cycle (uses subprocess - no persistent connection needed)
        faultier_frame = ttk.LabelFrame(frame, text="Faultier Power Control", padding=5)
        faultier_frame.pack(fill=tk.X, pady=3)

        ttk.Label(faultier_frame, text="Power cycle via MUX0 (subprocess)", font=("Arial", 8)).pack()

        # Power cycle button (always enabled - uses subprocess)
        self.power_cycle_btn = ttk.Button(faultier_frame, text="Power Cycle Target",
                                          command=self.power_cycle_target)
        self.power_cycle_btn.pack(fill=tk.X, pady=3)

        # Test button to verify power cycle works
        self.test_power_btn = ttk.Button(faultier_frame, text="Test Power Cycle",
                                         command=self.test_power_cycle)
        self.test_power_btn.pack(fill=tk.X, pady=3)

        # Faulty Cat EMP Device Connection
        faultycat_frame = ttk.LabelFrame(frame, text="Faulty Cat (EMP Generator)", padding=5)
        faultycat_frame.pack(fill=tk.X, pady=3)

        ttk.Label(faultycat_frame, text="Port:", font=("Arial", 8)).grid(row=0, column=0, sticky=tk.W, pady=1)
        self.faultycat_port_combo = ttk.Combobox(faultycat_frame, textvariable=self.faultycat_port, width=15, font=("Arial", 8))
        self.faultycat_port_combo.grid(row=0, column=1, pady=1)

        ttk.Label(faultycat_frame, text="Baud:", font=("Arial", 8)).grid(row=1, column=0, sticky=tk.W, pady=1)
        self.faultycat_baud_combo = ttk.Combobox(faultycat_frame, textvariable=self.faultycat_baudrate, width=15, font=("Arial", 8),
                                                  values=[115200, 57600, 38400, 19200, 9600])
        self.faultycat_baud_combo.grid(row=1, column=1, pady=1)

        self.faultycat_connect_btn = ttk.Button(faultycat_frame, text="Connect", command=self.connect_faultycat)
        self.faultycat_connect_btn.grid(row=2, column=0, columnspan=2, pady=3)

        self.faultycat_status = ttk.Label(faultycat_frame, text="Not Connected", foreground="gray", font=("Arial", 8))
        self.faultycat_status.grid(row=3, column=0, columnspan=2)

        # Pulse count control
        pulse_count_frame = ttk.Frame(faultycat_frame)
        pulse_count_frame.grid(row=4, column=0, columnspan=2, pady=2)

        ttk.Label(pulse_count_frame, text="Pulses:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.pulse_count_spinbox = ttk.Spinbox(pulse_count_frame, from_=1, to=100,
                                                textvariable=self.pulse_count, width=5, font=("Arial", 8))
        self.pulse_count_spinbox.pack(side=tk.LEFT, padx=5)

        # Faulty Cat control buttons
        faultycat_ctrl_frame = ttk.Frame(faultycat_frame)
        faultycat_ctrl_frame.grid(row=5, column=0, columnspan=2, pady=3)

        self.faultycat_arm_btn = ttk.Button(faultycat_ctrl_frame, text="Arm", width=8,
                                             command=self.arm_faultycat, state=tk.DISABLED)
        self.faultycat_arm_btn.pack(side=tk.LEFT, padx=2)

        self.faultycat_disarm_btn = ttk.Button(faultycat_ctrl_frame, text="Disarm", width=8,
                                                command=self.disarm_faultycat, state=tk.DISABLED)
        self.faultycat_disarm_btn.pack(side=tk.LEFT, padx=2)

        self.faultycat_pulse_btn = ttk.Button(faultycat_ctrl_frame, text="PULSE!", width=10,
                                               command=self.manual_pulse_faultycat, state=tk.DISABLED)
        self.faultycat_pulse_btn.pack(side=tk.LEFT, padx=2)

        self.faultycat_armed_label = ttk.Label(faultycat_frame, text="DISARMED", foreground="gray", font=("Arial", 8, "bold"))
        self.faultycat_armed_label.grid(row=6, column=0, columnspan=2)

        # Target Device Connection (via TI Debug Probe UART)
        target_frame = ttk.LabelFrame(frame, text="Target UART (TI Debug Probe)", padding=5)
        target_frame.pack(fill=tk.X, pady=3)

        ttk.Label(target_frame, text="Port:", font=("Arial", 8)).grid(row=0, column=0, sticky=tk.W, pady=1)
        self.target_port_combo = ttk.Combobox(target_frame, textvariable=self.target_port, width=15, font=("Arial", 8))
        self.target_port_combo.grid(row=0, column=1, pady=1)

        ttk.Label(target_frame, text="Baud:", font=("Arial", 8)).grid(row=1, column=0, sticky=tk.W, pady=1)
        self.target_baud_combo = ttk.Combobox(target_frame, textvariable=self.target_baudrate, width=15, font=("Arial", 8),
                                              values=[9600, 115200, 57600, 38400, 19200])
        self.target_baud_combo.grid(row=1, column=1, pady=1)

        self.target_connect_btn = ttk.Button(target_frame, text="Connect", command=self.connect_target)
        self.target_connect_btn.grid(row=2, column=0, columnspan=2, pady=3)

        self.target_status = ttk.Label(target_frame, text="Not Connected", foreground="gray", font=("Arial", 8))
        self.target_status.grid(row=3, column=0, columnspan=2)

        # Initialize port dropdowns with available devices
        self.refresh_serial_ports()

    def create_faultier_config_section(self, parent):
        # Faultier is used only for power cycling via MUX0
        # No configuration needed - it's automatic
        pass

    def create_target_status_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Target Device Status", padding=8)
        frame.pack(fill=tk.X, padx=5, pady=5)

        # Board alive/dead indicator (large and prominent)
        board_status_frame = ttk.Frame(frame)
        board_status_frame.pack(fill=tk.X, pady=5)

        ttk.Label(board_status_frame, text="Board:", font=("Arial", 10, "bold")).pack(side=tk.LEFT)
        self.board_alive_indicator = ttk.Label(board_status_frame, text="UNKNOWN",
                                                font=("Arial", 12, "bold"),
                                                foreground="white", background="gray",
                                                padding=(10, 5))
        self.board_alive_indicator.pack(side=tk.LEFT, padx=10)

        # Start/Stop monitor button
        self.board_monitor_btn = ttk.Button(board_status_frame, text="Start Monitor",
                                            command=self.toggle_board_monitor)
        self.board_monitor_btn.pack(side=tk.LEFT, padx=5)

        # State indicator
        state_frame = ttk.Frame(frame)
        state_frame.pack(fill=tk.X, pady=3)

        ttk.Label(state_frame, text="State:", font=("Arial", 9, "bold")).pack(side=tk.LEFT)
        self.state_indicator = ttk.Label(state_frame, text="UNKNOWN", font=("Arial", 9, "bold"),
                                         foreground="gray", background="lightgray", padding=5)
        self.state_indicator.pack(side=tk.LEFT, padx=10)

        # Current attempt info
        attempt_frame = ttk.Frame(frame)
        attempt_frame.pack(fill=tk.X, pady=3)

        ttk.Label(attempt_frame, text="Current Attempt:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.attempt_label = ttk.Label(attempt_frame, text="N/A", font=("Arial", 8))
        self.attempt_label.pack(side=tk.LEFT, padx=5)

        # Heartbeat info
        hb_frame = ttk.Frame(frame)
        hb_frame.pack(fill=tk.X, pady=3)

        ttk.Label(hb_frame, text="Heartbeats:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.heartbeat_label = ttk.Label(hb_frame, text="0", font=("Arial", 8))
        self.heartbeat_label.pack(side=tk.LEFT, padx=5)

        # Power cycles
        pc_frame = ttk.Frame(frame)
        pc_frame.pack(fill=tk.X, pady=3)

        ttk.Label(pc_frame, text="Power Cycles:", font=("Arial", 8)).pack(side=tk.LEFT)
        self.power_cycle_label = ttk.Label(pc_frame, text="0", font=("Arial", 8))
        self.power_cycle_label.pack(side=tk.LEFT, padx=5)

    def create_manual_control_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Manual Position Control", padding=8)
        frame.pack(fill=tk.X, padx=5, pady=5)

        # Position display
        pos_frame = ttk.Frame(frame)
        pos_frame.pack(fill=tk.X, pady=3)

        self.pos_label = ttk.Label(pos_frame, text="Position: X=0.00 Y=0.00 Z=0.00",
                                   font=("Arial", 9, "bold"))
        self.pos_label.pack()

        # Movement step size
        step_frame = ttk.Frame(frame)
        step_frame.pack(fill=tk.X, pady=3)

        ttk.Label(step_frame, text="Step (mm):", font=("Arial", 8)).pack(side=tk.LEFT)
        ttk.Entry(step_frame, textvariable=self.move_step, width=8, font=("Arial", 8)).pack(side=tk.LEFT, padx=5)

        # XY control pad
        xy_frame = ttk.Frame(frame)
        xy_frame.pack(pady=3)

        ttk.Button(xy_frame, text="Y+", width=6,
                   command=lambda: self.manual_move('Y', 1)).grid(row=0, column=1, padx=1, pady=1)

        ttk.Button(xy_frame, text="X-", width=6,
                   command=lambda: self.manual_move('X', -1)).grid(row=1, column=0, padx=1, pady=1)
        ttk.Button(xy_frame, text="Home", width=6,
                   command=self.home_printer).grid(row=1, column=1, padx=1, pady=1)
        ttk.Button(xy_frame, text="X+", width=6,
                   command=lambda: self.manual_move('X', 1)).grid(row=1, column=2, padx=1, pady=1)

        ttk.Button(xy_frame, text="Y-", width=6,
                   command=lambda: self.manual_move('Y', -1)).grid(row=2, column=1, padx=1, pady=1)

        # Z controls
        z_frame = ttk.Frame(frame)
        z_frame.pack(pady=3)

        ttk.Button(z_frame, text="Z+", width=6,
                   command=lambda: self.manual_move('Z', 1)).pack(side=tk.LEFT, padx=1)
        ttk.Button(z_frame, text="Z-", width=6,
                   command=lambda: self.manual_move('Z', -1)).pack(side=tk.LEFT, padx=1)

        # Chip corner setup frame
        corner_frame = ttk.LabelFrame(frame, text="Chip Area Setup", padding=8)
        corner_frame.pack(fill=tk.X, pady=5)

        self.set_origin_btn = ttk.Button(corner_frame,
                                         text="1. Set Bottom-Left Origin",
                                         command=self.set_chip_origin,
                                         style="Accent.TButton")
        self.set_origin_btn.pack(fill=tk.X, pady=2)

        self.origin_status = ttk.Label(corner_frame, text="Origin not set", foreground="red", font=("Arial", 8))
        self.origin_status.pack(pady=1)

        self.set_top_right_btn = ttk.Button(corner_frame,
                                            text="2. Mark Top-Right Corner",
                                            command=self.set_top_right_corner,
                                            state=tk.DISABLED,
                                            style="Accent.TButton")
        self.set_top_right_btn.pack(fill=tk.X, pady=2)

        self.top_right_status = ttk.Label(corner_frame, text="Top-right not set", foreground="red", font=("Arial", 8))
        self.top_right_status.pack(pady=1)

        self.area_info = ttk.Label(corner_frame, text="Scan Area: Not configured",
                                   font=("Arial", 8, "italic"))
        self.area_info.pack(pady=3)

    def create_scan_config_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Scan Configuration", padding=8)
        frame.pack(fill=tk.X, padx=5, pady=5)

        # Grid configuration
        grid_frame = ttk.LabelFrame(frame, text="Grid Settings", padding=5)
        grid_frame.pack(fill=tk.X, pady=3)

        config_items = [
            ("Step Size (mm):", self.step_size),
            ("Z Increment (mm):", self.z_increment),
            ("Max Z Height (mm):", self.max_z_height),
            ("Pulses per Location:", self.pulses_per_location)
        ]

        for i, (label, var) in enumerate(config_items):
            ttk.Label(grid_frame, text=label, font=("Arial", 8)).grid(row=i, column=0, sticky=tk.W, pady=1)
            ttk.Entry(grid_frame, textvariable=var, width=10, font=("Arial", 8)).grid(row=i, column=1, pady=1, padx=3)

        # Timing delays
        timing_frame = ttk.LabelFrame(frame, text="Timing Delays (ms)", padding=5)
        timing_frame.pack(fill=tk.X, pady=3)

        timing_items = [
            ("Between Pulses:", self.delay_between_pulses, "Capacitor recharge time"),
            ("After Move:", self.delay_after_move, "Settle time after positioning"),
            ("After Power Cycle:", self.delay_after_power_cycle, "Wait for board to boot"),
            ("Power Cycle Length:", self.power_cycle_duration, "How long to cut power"),
        ]

        for i, (label, var, tooltip) in enumerate(timing_items):
            ttk.Label(timing_frame, text=label, font=("Arial", 8)).grid(row=i, column=0, sticky=tk.W, pady=1)
            entry = ttk.Entry(timing_frame, textvariable=var, width=8, font=("Arial", 8))
            entry.grid(row=i, column=1, pady=1, padx=3)
            ttk.Label(timing_frame, text=f"({tooltip})", font=("Arial", 7), foreground="gray").grid(row=i, column=2, sticky=tk.W, padx=2)

        # Grid info and time estimate
        info_frame = ttk.Frame(frame)
        info_frame.pack(fill=tk.X, pady=3)

        self.grid_info = ttk.Label(info_frame, text="Grid: Configure chip area first", font=("Arial", 8))
        self.grid_info.pack()

        self.time_estimate = ttk.Label(info_frame, text="Estimated time: --", font=("Arial", 8, "bold"), foreground="blue")
        self.time_estimate.pack()

        ttk.Button(frame, text="Calculate Grid & Time",
                   command=self.update_grid_info).pack(fill=tk.X, pady=2)

    def create_scan_control_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Scan Control", padding=8)
        frame.pack(fill=tk.X, padx=5, pady=5)

        # EMP Scan (Faulty Cat) - primary scan mode
        self.start_emp_scan_btn = ttk.Button(frame, text="Start EMP Scan (Faulty Cat)",
                                              command=self.start_emp_scan,
                                              style="Accent.TButton",
                                              state=tk.DISABLED)
        self.start_emp_scan_btn.pack(fill=tk.X, pady=3)

        # Faultier Scan (trigger-based)
        self.start_scan_btn = ttk.Button(frame, text="Start Faultier Scan",
                                         command=self.start_scan,
                                         state=tk.DISABLED)
        self.start_scan_btn.pack(fill=tk.X, pady=3)

        self.stop_scan_btn = ttk.Button(frame, text="Stop Scan",
                                        command=self.stop_scan,
                                        state=tk.DISABLED)
        self.stop_scan_btn.pack(fill=tk.X, pady=3)

        self.reset_data_btn = ttk.Button(frame, text="Reset Data",
                                         command=self.reset_data)
        self.reset_data_btn.pack(fill=tk.X, pady=3)

        # Scan progress info
        self.scan_progress_label = ttk.Label(frame, text="Ready to scan", font=("Arial", 8))
        self.scan_progress_label.pack(fill=tk.X, pady=3)

    def create_status_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Scan Statistics", padding=10)
        frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.stats_text = tk.Text(frame, height=12, width=35, state=tk.DISABLED, font=("Courier", 8))
        self.stats_text.pack(fill=tk.BOTH, expand=True)

        self.update_statistics()

    def create_visualization_section(self, parent):
        # Create notebook for tabs
        self.viz_notebook = ttk.Notebook(parent)
        self.viz_notebook.pack(fill=tk.BOTH, expand=True)

        # Tab 1: Target Serial Monitor
        monitor_frame = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(monitor_frame, text="Target Monitor")

        # Serial monitor controls
        monitor_controls = ttk.Frame(monitor_frame)
        monitor_controls.pack(fill=tk.X, padx=5, pady=5)

        self.monitor_btn = ttk.Button(monitor_controls, text="Start Monitor",
                                      command=self.toggle_serial_monitor)
        self.monitor_btn.pack(side=tk.LEFT, padx=5)

        ttk.Button(monitor_controls, text="Clear",
                   command=self.clear_serial_monitor).pack(side=tk.LEFT, padx=5)

        self.autoscroll_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(monitor_controls, text="Auto-scroll",
                       variable=self.autoscroll_var).pack(side=tk.LEFT, padx=5)

        ttk.Checkbutton(monitor_controls, text="Log to file",
                       variable=self.serial_logging_enabled).pack(side=tk.LEFT, padx=5)

        ttk.Button(monitor_controls, text="Open Log Folder",
                   command=self.open_log_folder).pack(side=tk.LEFT, padx=5)

        # Log file status
        log_status_frame = ttk.Frame(monitor_frame)
        log_status_frame.pack(fill=tk.X, padx=5)

        self.log_file_label = ttk.Label(log_status_frame, text="Log: Not logging", font=("Arial", 7), foreground="gray")
        self.log_file_label.pack(side=tk.LEFT)

        # Serial monitor display
        monitor_display_frame = ttk.Frame(monitor_frame)
        monitor_display_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.serial_monitor = scrolledtext.ScrolledText(
            monitor_display_frame,
            wrap=tk.WORD,
            font=("Courier", 9),
            bg="#1e1e1e",
            fg="#00ff00",
            insertbackground="white"
        )
        self.serial_monitor.pack(fill=tk.BOTH, expand=True)

        # Configure tags for message highlighting
        self.serial_monitor.tag_config("timestamp", foreground="#888888")
        self.serial_monitor.tag_config("system", foreground="#00aaff")
        self.serial_monitor.tag_config("error", foreground="#ff0000")
        self.serial_monitor.tag_config("success", foreground="#00ff00", font=("Courier", 9, "bold"))
        self.serial_monitor.tag_config("heartbeat", foreground="#ffaa00")
        self.serial_monitor.tag_config("attempt", foreground="#aa00ff")

        # Add timestamp option
        timestamp_frame = ttk.Frame(monitor_frame)
        timestamp_frame.pack(fill=tk.X, padx=5, pady=5)

        self.timestamp_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(timestamp_frame, text="Show timestamps",
                       variable=self.timestamp_var).pack(side=tk.LEFT)

        # Tab 2: Success Rate Graph
        graph_frame = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(graph_frame, text="Success Rate Graph")

        self.fig = Figure(figsize=(7, 6))
        self.ax_success = self.fig.add_subplot(111)
        self.ax_success.set_title("Glitch Success Rate by Location", fontsize=10)
        self.ax_success.set_xlabel("Location Index", fontsize=8)
        self.ax_success.set_ylabel("Success Rate (%)", fontsize=8)
        self.ax_success.tick_params(labelsize=7)
        self.ax_success.grid(True, alpha=0.3)

        self.fig.tight_layout(pad=2.0)

        self.canvas = FigureCanvasTkAgg(self.fig, graph_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Tab 3: 3D Visualization
        viz3d_frame = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(viz3d_frame, text="3D View")

        self.fig_3d = Figure(figsize=(7, 6))
        self.ax_3d = self.fig_3d.add_subplot(111, projection='3d')
        self.ax_3d.set_title("3D Scan Visualization", fontsize=10)
        self.ax_3d.set_xlabel("X (mm)", fontsize=8)
        self.ax_3d.set_ylabel("Y (mm)", fontsize=8)
        self.ax_3d.set_zlabel("Z (mm)", fontsize=8)
        self.ax_3d.tick_params(labelsize=7)

        self.canvas_3d = FigureCanvasTkAgg(self.fig_3d, viz3d_frame)
        self.canvas_3d.draw()
        self.canvas_3d.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Tab 4: Controller Log
        log_frame = ttk.Frame(self.viz_notebook)
        self.viz_notebook.add(log_frame, text="Controller Log")

        self.controller_log = scrolledtext.ScrolledText(
            log_frame,
            wrap=tk.WORD,
            font=("Courier", 9),
            bg="#1a1a2e",
            fg="#eee",
            insertbackground="white"
        )
        self.controller_log.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    # ======================== CALLBACKS ========================

    def on_target_state_change(self, new_state: TargetState):
        """Callback when target state changes"""
        self.root.after(0, self.update_state_indicator, new_state)

    def on_controller_message(self, message: str):
        """Callback for controller log messages"""
        self.root.after(0, self.append_to_controller_log, message)

    def update_state_indicator(self, state: TargetState):
        """Update the state indicator widget"""
        state_colors = {
            TargetState.UNKNOWN: ("gray", "lightgray"),
            TargetState.READY: ("white", "green"),
            TargetState.ATTEMPT_STARTED: ("black", "yellow"),
            TargetState.RUNNING: ("white", "blue"),
            TargetState.GLITCH_SUCCESS: ("black", "lime"),
            TargetState.CRASHED: ("white", "red"),
            TargetState.POWER_CYCLING: ("black", "orange"),
        }

        fg, bg = state_colors.get(state, ("gray", "lightgray"))
        self.state_indicator.config(text=state.name, foreground=fg, background=bg)

    def append_to_controller_log(self, message: str):
        """Append message to controller log"""
        try:
            self.controller_log.config(state=tk.NORMAL)
            timestamp = time.strftime("[%H:%M:%S] ")
            self.controller_log.insert(tk.END, timestamp, "timestamp")
            self.controller_log.insert(tk.END, message + "\n")
            self.controller_log.see(tk.END)
            self.controller_log.config(state=tk.DISABLED)
        except:
            pass

    # ======================== FAULTIER HANDLERS ========================

    def test_power_cycle(self):
        """Test the power cycle to verify it works"""
        self.append_to_controller_log("Testing power cycle...")
        self.test_power_btn.config(state=tk.DISABLED)
        self.root.update()

        success, msg = self.controller.power_cycle_target()
        self.append_to_controller_log(msg)

        self.test_power_btn.config(state=tk.NORMAL)

        if success:
            messagebox.showinfo("Power Cycle Test",
                              f"Power cycle command sent.\n\n{msg}\n\nDid your board reset?")
        else:
            messagebox.showerror("Power Cycle Failed", msg)

    def power_cycle_target(self):
        if messagebox.askyesno("Power Cycle", "Power cycle the target via MUX0?"):
            self.power_cycle_btn.config(state=tk.DISABLED)
            self.root.update()

            success, msg = self.controller.power_cycle_target()
            self.append_to_controller_log(msg)

            self.power_cycle_btn.config(state=tk.NORMAL)
            if not success:
                messagebox.showerror("Error", msg)

    # ======================== SERIAL MONITOR ========================

    def toggle_serial_monitor(self):
        if not self.monitor_running:
            if not self.controller.target_connected:
                messagebox.showwarning("Not Connected",
                                     "Please connect to target device first")
                return
            self.start_serial_monitor()
        else:
            self.stop_serial_monitor()

    def start_serial_monitor(self):
        self.monitor_running = True
        self.monitor_btn.config(text="Stop Monitor")
        self.append_to_monitor("=== Serial Monitor Started ===\n", "system")

        self.monitor_thread = threading.Thread(target=self.serial_monitor_loop, daemon=True)
        self.monitor_thread.start()

    def stop_serial_monitor(self):
        self.monitor_running = False
        self.monitor_btn.config(text="Start Monitor")
        self.append_to_monitor("=== Serial Monitor Stopped ===\n", "system")

    def serial_monitor_loop(self):
        """Continuously read from target serial response queue.

        Note: If board_monitor is running, it handles queue reading and forwards
        to serial monitor. This loop only runs when board monitor is NOT active.
        """
        while self.monitor_running:
            # If board monitor is running, it handles queue reading
            # and forwards to serial monitor - we just wait
            if self.board_monitor_running:
                time.sleep(0.1)
                continue

            try:
                # Read from the controller's response queue
                line = self.controller._read_target_line(timeout=0.1)
                if line:
                    # Update heartbeat tracking
                    self.last_heartbeat_time = time.time()

                    # Determine message type for highlighting
                    msg_type = "data"
                    if "SUCCESS" in line:
                        msg_type = "success"
                    elif "HB:" in line:
                        msg_type = "heartbeat"
                    elif "ATTEMPT:" in line:
                        msg_type = "attempt"
                    elif "ERROR" in line or "CRASH" in line:
                        msg_type = "error"

                    self.root.after(0, self.append_to_monitor, line + "\n", msg_type)

            except Exception as e:
                self.root.after(0, self.append_to_monitor,
                              f"[ERROR] {str(e)}\n", "error")
                time.sleep(0.5)

    def append_to_monitor(self, text, msg_type="data"):
        """Append text to serial monitor with formatting and log to file"""
        try:
            self.serial_monitor.config(state=tk.NORMAL)

            if self.timestamp_var.get() and msg_type not in ("system",):
                timestamp = time.strftime("[%H:%M:%S] ")
                self.serial_monitor.insert(tk.END, timestamp, "timestamp")

            self.serial_monitor.insert(tk.END, text, msg_type)

            if self.autoscroll_var.get():
                self.serial_monitor.see(tk.END)

            self.serial_monitor.config(state=tk.DISABLED)

            # Write to log file (strip the newline since write_to_serial_log doesn't add one)
            if msg_type != "system":
                self.write_to_serial_log(text)
        except:
            pass

    def clear_serial_monitor(self):
        self.serial_monitor.config(state=tk.NORMAL)
        self.serial_monitor.delete(1.0, tk.END)
        self.serial_monitor.config(state=tk.DISABLED)

    # ======================== SERIAL LOGGING ========================

    def open_log_folder(self):
        """Open the serial logs folder"""
        if not os.path.exists(self.serial_log_dir):
            os.makedirs(self.serial_log_dir)
        import subprocess
        subprocess.Popen(['xdg-open', self.serial_log_dir])

    def start_serial_log(self):
        """Start a new serial log file"""
        if not os.path.exists(self.serial_log_dir):
            os.makedirs(self.serial_log_dir)

        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.serial_log_path = os.path.join(self.serial_log_dir, f"serial_log_{timestamp}.txt")
        self.serial_log_file = open(self.serial_log_path, 'w')
        self.serial_log_size = 0

        # Write header
        header = f"=== Serial Log Started: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n"
        self.serial_log_file.write(header)
        self.serial_log_file.flush()
        self.serial_log_size = len(header)

        self.log_file_label.config(text=f"Log: {os.path.basename(self.serial_log_path)}", foreground="green")

    def stop_serial_log(self):
        """Close the current serial log file"""
        if self.serial_log_file:
            self.serial_log_file.write(f"\n=== Serial Log Ended: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')} ===\n")
            self.serial_log_file.close()
            self.serial_log_file = None
        self.log_file_label.config(text="Log: Not logging", foreground="gray")

    def write_to_serial_log(self, text):
        """Write text to serial log file, rotating if needed"""
        if not self.serial_logging_enabled.get():
            return

        # Start log file if not already open
        if self.serial_log_file is None:
            self.start_serial_log()

        # Check if we need to rotate
        if self.serial_log_size >= MAX_LOG_FILE_SIZE:
            self.append_to_controller_log(f"Log file reached {MAX_LOG_FILE_SIZE // (1024*1024)}MB, rotating...")
            self.stop_serial_log()
            self.start_serial_log()

        # Write to file
        try:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            log_line = f"[{timestamp}] {text}"
            self.serial_log_file.write(log_line)
            self.serial_log_file.flush()
            self.serial_log_size += len(log_line)
        except Exception as e:
            print(f"Error writing to log: {e}")

    # ======================== BOARD STATUS MONITORING ========================

    def toggle_board_monitor(self):
        """Toggle the board alive/dead monitoring thread"""
        if not self.board_monitor_running:
            if not self.controller.target_connected:
                messagebox.showwarning("Not Connected",
                                     "Please connect to target device first")
                return
            self.start_board_monitor()
        else:
            self.stop_board_monitor()

    def start_board_monitor(self):
        """Start background thread that monitors board heartbeats"""
        self.board_monitor_running = True
        self.last_heartbeat_time = time.time()
        self.board_monitor_btn.config(text="Stop Monitor")
        self.append_to_controller_log("Board monitoring started")

        self.board_monitor_thread = threading.Thread(target=self.board_monitor_loop, daemon=True)
        self.board_monitor_thread.start()

    def stop_board_monitor(self):
        """Stop the board monitoring thread"""
        self.board_monitor_running = False
        self.board_monitor_btn.config(text="Start Monitor")
        self.append_to_controller_log("Board monitoring stopped")

    def board_monitor_loop(self):
        """
        Background loop that continuously monitors for heartbeats.
        Updates board_alive status based on whether heartbeats are received.
        """
        while self.board_monitor_running:
            try:
                # Read from the controller's response queue
                line = self.controller._read_target_line(timeout=0.1)
                if line:
                    current_time = time.time()

                    # Check for heartbeat messages
                    if "HB:" in line:
                        self.last_heartbeat_time = current_time
                        if not self.board_alive:
                            self.board_alive = True
                            self.root.after(0, self.update_board_status, True)

                    # Also consider READY, ATTEMPT, START as signs of life
                    elif any(marker in line for marker in ["READY", "ATTEMPT:", "START", "SUCCESS"]):
                        self.last_heartbeat_time = current_time
                        if not self.board_alive:
                            self.board_alive = True
                            self.root.after(0, self.update_board_status, True)

                    # Pass to serial monitor if running
                    if self.monitor_running:
                        msg_type = "data"
                        if "SUCCESS" in line:
                            msg_type = "success"
                        elif "HB:" in line:
                            msg_type = "heartbeat"
                        elif "ATTEMPT:" in line:
                            msg_type = "attempt"
                        elif "ERROR" in line or "CRASH" in line:
                            msg_type = "error"
                        self.root.after(0, self.append_to_monitor, line + "\n", msg_type)

                # Check if board has timed out (no heartbeat recently)
                time_since_heartbeat = time.time() - self.last_heartbeat_time
                if time_since_heartbeat > self.heartbeat_timeout_sec:
                    if self.board_alive:
                        self.board_alive = False
                        self.root.after(0, self.update_board_status, False)

            except Exception as e:
                self.root.after(0, self.append_to_controller_log,
                              f"Board monitor error: {str(e)}")
                time.sleep(0.5)

            time.sleep(0.01)

    def update_board_status(self, alive: bool):
        """Update the board status indicator in the GUI"""
        if alive:
            self.board_alive_indicator.config(text="ALIVE", background="green", foreground="white")
        else:
            self.board_alive_indicator.config(text="DEAD", background="red", foreground="white")

    def check_board_alive_after_pulse(self):
        """
        Check if board is alive after firing pulses.
        Waits for heartbeats or any response from the board.
        """
        self.append_to_controller_log("Checking board status after pulse...")
        check_start = time.time()
        check_timeout = 3.0  # Wait up to 3 seconds for response

        while time.time() - check_start < check_timeout:
            line = self.controller._read_target_line(timeout=0.1)
            if line:
                self.last_heartbeat_time = time.time()
                if "HB:" in line or "READY" in line or "ATTEMPT:" in line or "SUCCESS" in line:
                    self.board_alive = True
                    self.root.after(0, self.update_board_status, True)
                    self.append_to_controller_log(f"Board response: {line}")

                    # Check for successful glitch
                    if "SUCCESS" in line:
                        self.append_to_controller_log("*** GLITCH SUCCESS DETECTED! ***")
                        messagebox.showinfo("Glitch Detected!", f"Board response indicates successful glitch!\n\n{line}")
                    return True

        # No response - board may be dead
        self.board_alive = False
        self.root.after(0, self.update_board_status, False)
        self.append_to_controller_log("No response from board - may be crashed")
        return False

    # ======================== CONNECTION HANDLERS ========================

    def refresh_serial_ports(self):
        """Refresh the list of available serial ports in all dropdowns."""
        ports = get_available_serial_ports()

        # If no ports found, add placeholders
        if not ports:
            ports = ['/dev/ttyACM0', '/dev/ttyACM1', '/dev/ttyACM2', '/dev/ttyUSB0']

        # Update all port combos
        self.printer_port_combo['values'] = ports
        self.faultycat_port_combo['values'] = ports
        self.target_port_combo['values'] = ports

    def connect_printer(self):
        if not self.controller.printer_connected:
            success, msg = self.controller.connect_printer(
                self.printer_port.get(),
                self.printer_baudrate.get()
            )
            if success:
                self.printer_status.config(text="Connected", foreground="green")
                self.printer_connect_btn.config(text="Disconnect")
                messagebox.showinfo("Success", msg)
            else:
                messagebox.showerror("Connection Error", msg)
        else:
            success, msg = self.controller.disconnect_printer()
            self.printer_status.config(text="Disconnected", foreground="red")
            self.printer_connect_btn.config(text="Connect")

    def connect_faultycat(self):
        """Connect to Faulty Cat EMP generator"""
        if not self.controller.faultycat_connected:
            success, msg = self.controller.connect_faultycat(
                self.faultycat_port.get(),
                self.faultycat_baudrate.get()
            )
            if success:
                self.faultycat_status.config(text="Connected", foreground="green")
                self.faultycat_connect_btn.config(text="Disconnect")
                self.faultycat_arm_btn.config(state=tk.NORMAL)
                self.faultycat_disarm_btn.config(state=tk.NORMAL)
                self.faultycat_pulse_btn.config(state=tk.NORMAL)
                self.append_to_controller_log(msg)
            else:
                messagebox.showerror("Connection Error", msg)
        else:
            success, msg = self.controller.disconnect_faultycat()
            self.faultycat_status.config(text="Not Connected", foreground="gray")
            self.faultycat_connect_btn.config(text="Connect")
            self.faultycat_arm_btn.config(state=tk.DISABLED)
            self.faultycat_disarm_btn.config(state=tk.DISABLED)
            self.faultycat_pulse_btn.config(state=tk.DISABLED)
            self.faultycat_armed_label.config(text="DISARMED", foreground="gray")

    def arm_faultycat(self):
        """Arm the Faulty Cat (charge capacitor)"""
        success, msg = self.controller.arm_faultycat()
        if success:
            self.faultycat_armed_label.config(text="ARMED", foreground="red")
            self.append_to_controller_log(msg)
        else:
            messagebox.showerror("Error", msg)

    def disarm_faultycat(self):
        """Disarm the Faulty Cat"""
        success, msg = self.controller.disarm_faultycat()
        if success:
            self.faultycat_armed_label.config(text="DISARMED", foreground="gray")
            self.append_to_controller_log(msg)
        else:
            messagebox.showerror("Error", msg)

    def manual_pulse_faultycat(self):
        """Fire manual pulse(s) (arms first if needed) and check board status"""
        if not self.controller.faultycat_connected:
            messagebox.showwarning("Not Connected", "Please connect Faulty Cat first")
            return

        num_pulses = self.pulse_count.get()

        # Confirm before firing
        if not messagebox.askyesno("Fire EMP Pulse",
                                    f"This will charge and fire {num_pulses} EMP pulse(s).\n\n"
                                    "Make sure the probe is in position!\n\n"
                                    "Continue?"):
            return

        # Disable controls during pulse sequence
        self.faultycat_pulse_btn.config(state=tk.DISABLED)
        self.faultycat_arm_btn.config(state=tk.DISABLED)
        self.faultycat_disarm_btn.config(state=tk.DISABLED)
        self.root.update()

        # Run pulse sequence in background thread to avoid blocking GUI
        threading.Thread(target=self._pulse_sequence_thread,
                        args=(num_pulses,), daemon=True).start()

    def _pulse_sequence_thread(self, num_pulses: int):
        """Background thread to fire pulse sequence and check board status"""
        pulses_fired = 0
        glitch_detected = False

        try:
            for i in range(num_pulses):
                self.root.after(0, lambda n=i+1, t=num_pulses:
                              self.faultycat_armed_label.config(
                                  text=f"PULSE {n}/{t}", foreground="orange"))

                success, msg = self.controller.pulse_faultycat()

                if success:
                    pulses_fired += 1
                    self.root.after(0, self.append_to_controller_log, f"Pulse {i+1}/{num_pulses} fired")

                    # Brief delay between pulses (allow capacitor recharge)
                    if i < num_pulses - 1:
                        time.sleep(0.3)  # 300ms between pulses for recharge
                else:
                    self.root.after(0, self.append_to_controller_log, f"Pulse {i+1} failed: {msg}")
                    break

            # Update label to show checking status
            self.root.after(0, lambda: self.faultycat_armed_label.config(
                text="CHECKING...", foreground="blue"))

            # Check board status after pulse sequence
            self.root.after(0, self.append_to_controller_log,
                          f"Fired {pulses_fired}/{num_pulses} pulses, checking board status...")

            # Wait and check for board response
            check_start = time.time()
            check_timeout = 3.0
            response_received = False

            while time.time() - check_start < check_timeout:
                line = self.controller._read_target_line(timeout=0.1)
                if line:
                    self.last_heartbeat_time = time.time()
                    response_received = True

                    # Log the response
                    self.root.after(0, self.append_to_controller_log, f"Board: {line}")

                    # Check for successful glitch
                    if "SUCCESS" in line:
                        glitch_detected = True
                        self.root.after(0, self.append_to_controller_log,
                                       "*** GLITCH SUCCESS DETECTED! ***")
                        self.root.after(0, lambda: messagebox.showinfo(
                            "Glitch Detected!",
                            f"Successful glitch after {pulses_fired} pulse(s)!\n\n{line}"))
                        break

                    # If we see heartbeat or ready, board is still alive
                    if "HB:" in line or "READY" in line:
                        self.root.after(0, lambda: self.update_board_status(True))

            # Update final board status
            if response_received:
                self.board_alive = True
                self.root.after(0, lambda: self.update_board_status(True))
                if not glitch_detected:
                    self.root.after(0, self.append_to_controller_log,
                                   "Board is alive - no glitch effect detected")
            else:
                self.board_alive = False
                self.root.after(0, lambda: self.update_board_status(False))
                self.root.after(0, self.append_to_controller_log,
                               "No response from board - may have crashed!")

        except Exception as e:
            self.root.after(0, self.append_to_controller_log, f"Pulse sequence error: {str(e)}")

        finally:
            # Re-enable controls
            self.root.after(0, lambda: self.faultycat_pulse_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.faultycat_arm_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.faultycat_disarm_btn.config(state=tk.NORMAL))
            self.root.after(0, lambda: self.faultycat_armed_label.config(
                text="DISARMED", foreground="gray"))

    def connect_target(self):
        """Connect to target UART via TI debug probe"""
        if not self.controller.target_connected:
            success, msg = self.controller.connect_target(
                self.target_port.get(),
                self.target_baudrate.get()
            )
            if success:
                self.target_status.config(text="Connected", foreground="green")
                self.target_connect_btn.config(text="Disconnect")
                self.append_to_controller_log(msg)

                # Auto-start board monitoring
                if not self.board_monitor_running:
                    self.start_board_monitor()
            else:
                messagebox.showerror("Connection Error", msg)
        else:
            # Stop board monitor before disconnecting
            if self.board_monitor_running:
                self.stop_board_monitor()

            success, msg = self.controller.disconnect_target()
            self.target_status.config(text="Not Connected", foreground="gray")
            self.target_connect_btn.config(text="Connect")
            self.board_alive_indicator.config(text="UNKNOWN", background="gray")

    # ======================== MOVEMENT HANDLERS ========================

    def manual_move(self, axis, direction):
        if not self.controller.printer_connected:
            messagebox.showwarning("Not Connected", "Please connect to printer first")
            return

        distance = self.move_step.get() * direction
        success, msg = self.controller.move_relative(axis, distance)

        if success:
            self.update_position_display()
        else:
            messagebox.showerror("Movement Error", msg)

    def home_printer(self):
        if not self.controller.printer_connected:
            messagebox.showwarning("Not Connected", "Please connect to printer first")
            return

        if messagebox.askyesno("Home Printer", "Home all axes? This will move the printer."):
            success, msg = self.controller.home_all_axes()
            if success:
                self.update_position_display()
            else:
                messagebox.showerror("Homing Error", msg)

    def set_chip_origin(self):
        if not self.controller.printer_connected:
            messagebox.showwarning("Not Connected", "Please connect to printer first")
            return

        if messagebox.askyesno("Set Origin",
                               "Set current position as chip origin (0,0,0)?\n\n"
                               "Make sure the probe is at the BOTTOM-LEFT corner\n"
                               "of the chip and Z is at the chip surface."):
            success, msg = self.controller.set_origin()
            if success:
                self.origin_status.config(text="Origin set at (0, 0, 0)", foreground="green")
                self.set_top_right_btn.config(state=tk.NORMAL)
                self.update_position_display()
                self.update_area_info()
                messagebox.showinfo("Success", "Origin set! Now move probe to TOP-RIGHT corner and mark it.")
            else:
                messagebox.showerror("Error", msg)

    def set_top_right_corner(self):
        if not self.controller.printer_connected:
            messagebox.showwarning("Not Connected", "Please connect to printer first")
            return

        if messagebox.askyesno("Set Top-Right Corner",
                               f"Mark current position as top-right corner?\n\n"
                               f"Current position:\n"
                               f"X: {self.controller.current_x:.2f} mm\n"
                               f"Y: {self.controller.current_y:.2f} mm\n"
                               f"Z: {self.controller.current_z:.2f} mm\n\n"
                               f"This will define the scan area."):
            success, msg = self.controller.set_top_right_corner()
            if success:
                self.top_right_status.config(
                    text=f"Top-right set at ({self.controller.top_right_x:.2f}, {self.controller.top_right_y:.2f})",
                    foreground="green"
                )
                self.start_scan_btn.config(state=tk.NORMAL)
                self.start_emp_scan_btn.config(state=tk.NORMAL)
                self.update_area_info()
                self.update_grid_info()
                messagebox.showinfo("Success",
                                   f"Top-right corner marked!\n\n"
                                   f"Scan area: {self.controller.top_right_x:.2f} x {self.controller.top_right_y:.2f} mm\n\n"
                                   f"You can now configure and start the scan.")
            else:
                messagebox.showerror("Error", msg)

    def update_position_display(self):
        self.pos_label.config(
            text=f"Position: X={self.controller.current_x:.2f} Y={self.controller.current_y:.2f} Z={self.controller.current_z:.2f}"
        )

    def update_area_info(self):
        if self.controller.origin_set and self.controller.top_right_set:
            width = self.controller.top_right_x
            height = self.controller.top_right_y
            self.area_info.config(
                text=f"Scan Area: {width:.2f} mm x {height:.2f} mm",
                foreground="green"
            )
        elif self.controller.origin_set:
            self.area_info.config(
                text="Scan Area: Origin set, waiting for top-right corner",
                foreground="orange"
            )
        else:
            self.area_info.config(
                text="Scan Area: Not configured",
                foreground="red"
            )

    def update_grid_info(self):
        grid = self.controller.calculate_scan_grid(
            self.step_size.get(),
            self.z_increment.get(),
            self.max_z_height.get()
        )

        if grid:
            total_locations = grid['total_points']
            pulses_per_loc = self.pulses_per_location.get()
            total_pulses = total_locations * pulses_per_loc

            self.grid_info.config(
                text=f"Grid: {grid['x_steps']} x {grid['y_steps']} x {grid['z_steps']} = {total_locations} locations | {total_pulses} pulses"
            )

            # Calculate time estimate
            delay_pulse = self.delay_between_pulses.get() / 1000.0  # ms to sec
            delay_move = self.delay_after_move.get() / 1000.0
            delay_power = self.delay_after_power_cycle.get() / 1000.0

            # Time per location:
            # - Movement time (~0.5s average)
            # - Settle time after move
            # - Pulses * (pulse_time + delay_between_pulses)
            # - Occasional power cycles (estimate 10% crash rate)
            move_time = 0.5  # average movement time in seconds
            pulse_time = 0.1  # time to fire one pulse

            time_per_pulse = pulse_time + delay_pulse
            time_per_location = delay_move + (pulses_per_loc * time_per_pulse) + move_time

            # Add 10% for power cycles (crashes)
            crash_overhead = 0.10 * total_locations * delay_power

            total_seconds = (total_locations * time_per_location) + crash_overhead

            # Format time
            hours = int(total_seconds // 3600)
            minutes = int((total_seconds % 3600) // 60)
            seconds = int(total_seconds % 60)

            if hours > 0:
                time_str = f"{hours}h {minutes}m {seconds}s"
            elif minutes > 0:
                time_str = f"{minutes}m {seconds}s"
            else:
                time_str = f"{seconds}s"

            self.time_estimate.config(text=f"Estimated time: {time_str}")
        else:
            self.grid_info.config(text="Grid: Configure chip area first")
            self.time_estimate.config(text="Estimated time: --")

    # ======================== SCAN HANDLERS ========================

    def start_emp_scan(self):
        """Start EMP scan using Faulty Cat pulses"""
        if not self.controller.printer_connected:
            messagebox.showwarning("Not Connected", "Please connect to printer first")
            return

        if not self.controller.faultycat_connected:
            messagebox.showwarning("Not Connected", "Please connect to Faulty Cat first")
            return

        if not self.controller.target_connected:
            messagebox.showwarning("Not Connected",
                                  "Please connect to target UART for response monitoring")
            return

        if not self.controller.origin_set or not self.controller.top_right_set:
            messagebox.showwarning("Setup Incomplete",
                                  "Please set both origin and top-right corner first!")
            return

        grid = self.controller.calculate_scan_grid(
            self.step_size.get(),
            self.z_increment.get(),
            self.max_z_height.get()
        )

        total_pulses = grid['total_points'] * self.pulses_per_location.get()

        if not messagebox.askyesno("Start EMP Scan",
                                    f"Start EMP scan with Faulty Cat?\n\n"
                                    f"Scan area: {self.controller.top_right_x:.2f} x {self.controller.top_right_y:.2f} mm\n"
                                    f"Grid: {grid['x_steps']} x {grid['y_steps']} x {grid['z_steps']}\n"
                                    f"Total locations: {grid['total_points']}\n"
                                    f"Pulses per location: {self.pulses_per_location.get()}\n"
                                    f"Total EMP pulses: {total_pulses}\n\n"
                                    f"Power cycle on crash: YES (via MUX0)\n"
                                    f"Skipped instruction detection: YES"):
            return

        # Disable scan buttons
        self.start_scan_btn.config(state=tk.DISABLED)
        self.start_emp_scan_btn.config(state=tk.DISABLED)
        self.stop_scan_btn.config(state=tk.NORMAL)

        # Start EMP scan thread
        self.scan_thread = threading.Thread(target=self.run_emp_scan_thread, daemon=True)
        self.scan_thread.start()

    def run_emp_scan_thread(self):
        """Run EMP scan in background thread"""
        def progress_callback(current, total, x, y, z, location):
            # Update progress label
            self.root.after(0, lambda: self.scan_progress_label.config(
                text=f"Scanning: {current}/{total} ({current*100//total}%) - ({x:.2f}, {y:.2f}, {z:.2f})"))

            # Update displays
            self.root.after(0, self.update_position_display)
            self.root.after(0, self.update_visualization)
            self.root.after(0, self.update_statistics)

            # Log significant events
            if location.glitch_count > 0:
                self.root.after(0, self.append_to_controller_log,
                              f"GLITCH at ({x:.2f}, {y:.2f}, {z:.2f})!")
            if location.skipped_count > 0:
                self.root.after(0, self.append_to_controller_log,
                              f"Skipped at ({x:.2f}, {y:.2f}, {z:.2f})")
            if location.crash_count > 0:
                self.root.after(0, self.append_to_controller_log,
                              f"Crash at ({x:.2f}, {y:.2f}, {z:.2f}) - power cycled")

        success, msg = self.controller.run_emp_scan(
            self.step_size.get(),
            self.z_increment.get(),
            self.max_z_height.get(),
            self.pulses_per_location.get(),
            delay_between_pulses_ms=self.delay_between_pulses.get(),
            delay_after_move_ms=self.delay_after_move.get(),
            delay_after_power_cycle_ms=self.delay_after_power_cycle.get(),
            power_cycle_duration_ms=self.power_cycle_duration.get(),
            progress_callback=progress_callback
        )

        # Re-enable buttons
        self.root.after(0, lambda: self.start_scan_btn.config(state=tk.NORMAL))
        self.root.after(0, lambda: self.start_emp_scan_btn.config(state=tk.NORMAL))
        self.root.after(0, lambda: self.stop_scan_btn.config(state=tk.DISABLED))
        self.root.after(0, lambda: self.scan_progress_label.config(text="Scan complete"))

        if success:
            self.root.after(0, self.show_heatmaps_and_recommendation)
        else:
            self.root.after(0, lambda: messagebox.showerror("EMP Scan Error", msg))

    def start_scan(self):
        if not self.controller.printer_connected:
            messagebox.showwarning("Not Connected", "Please connect to printer first")
            return

        if not self.controller.origin_set or not self.controller.top_right_set:
            messagebox.showwarning("Setup Incomplete",
                                  "Please set both origin and top-right corner first!")
            return

        grid = self.controller.calculate_scan_grid(
            self.step_size.get(),
            self.z_increment.get(),
            self.max_z_height.get()
        )

        total_pulses = grid['total_points'] * self.pulses_per_location.get()

        faultier_status = "Connected" if self.controller.faultier_connected else "NOT CONNECTED (simulated results)"
        target_status = "Connected" if self.controller.target_connected else "NOT CONNECTED"

        if not messagebox.askyesno("Start Scan",
                                    f"Start EMFI scan?\n\n"
                                    f"Scan area: {self.controller.top_right_x:.2f} x {self.controller.top_right_y:.2f} mm\n"
                                    f"Grid: {grid['x_steps']} x {grid['y_steps']} x {grid['z_steps']}\n"
                                    f"Total locations: {grid['total_points']}\n"
                                    f"Total EMFI pulses: {total_pulses}\n\n"
                                    f"Faultier: {faultier_status}\n"
                                    f"Target: {target_status}"):
            return

        self.start_scan_btn.config(state=tk.DISABLED)
        self.stop_scan_btn.config(state=tk.NORMAL)

        self.scan_thread = threading.Thread(target=self.run_scan_thread, daemon=True)
        self.scan_thread.start()

    def run_scan_thread(self):
        """Run scan in background thread"""
        def progress_callback(current, total, x, y, z, location):
            self.root.after(0, self.update_position_display)
            self.root.after(0, self.update_visualization)
            self.root.after(0, self.update_statistics)

        success, msg = self.controller.run_scan(
            self.step_size.get(),
            self.z_increment.get(),
            self.max_z_height.get(),
            self.pulses_per_location.get(),
            progress_callback
        )

        self.root.after(0, lambda: self.start_scan_btn.config(state=tk.NORMAL))
        self.root.after(0, lambda: self.stop_scan_btn.config(state=tk.DISABLED))

        if success:
            self.root.after(0, self.show_heatmaps_and_recommendation)
        else:
            self.root.after(0, lambda: messagebox.showerror("Scan Error", msg))

    def stop_scan(self):
        self.controller.stop_scan()
        self.start_scan_btn.config(state=tk.NORMAL)
        self.start_emp_scan_btn.config(state=tk.NORMAL)
        self.stop_scan_btn.config(state=tk.DISABLED)
        self.scan_progress_label.config(text="Scan stopped")

    def reset_data(self):
        if messagebox.askyesno("Reset Data", "Clear all scan data and statistics?"):
            self.controller.reset_data()

            # Clear 2D graph
            self.ax_success.clear()
            self.ax_success.set_xlabel("Location Index", fontsize=8)
            self.ax_success.set_ylabel("Success Rate (%)", fontsize=8)
            self.ax_success.set_title("Glitch Success Rate by Location", fontsize=10)
            self.ax_success.grid(True, alpha=0.3)
            self.ax_success.set_ylim([0, 100])

            self.fig.tight_layout(pad=2.0)
            self.canvas.draw()

            # Clear 3D graph
            self.ax_3d.clear()
            self.ax_3d.set_xlabel("X (mm)", fontsize=8)
            self.ax_3d.set_ylabel("Y (mm)", fontsize=8)
            self.ax_3d.set_zlabel("Z (mm)", fontsize=8)
            self.ax_3d.set_title("3D Scan Visualization", fontsize=10)
            self.fig_3d.tight_layout()
            self.canvas_3d.draw()

            self.update_statistics()
            self.scan_progress_label.config(text="Ready to scan")

    # ======================== PERIODIC UPDATES ========================

    def update_ui_periodically(self):
        """Update UI elements periodically"""
        # Don't update if closing
        if self.closing:
            return

        try:
            # Update heartbeat and attempt counters
            self.heartbeat_label.config(text=str(self.controller.heartbeat_count))
            self.attempt_label.config(text=str(self.controller.current_attempt_number))
            self.power_cycle_label.config(text=str(self.controller.total_power_cycles))

            # Schedule next update
            self.root.after(100, self.update_ui_periodically)
        except tk.TclError:
            # Window was destroyed, stop updating
            pass

    # ======================== VISUALIZATION ========================

    def update_visualization(self):
        if not self.controller.scan_locations:
            return

        # Update 2D success rate graph
        self.ax_success.clear()

        glitch_rates = [loc.glitch_rate * 100 for loc in self.controller.scan_locations]
        skipped_rates = [loc.skipped_rate * 100 for loc in self.controller.scan_locations]
        crash_rates = [loc.crash_rate * 100 for loc in self.controller.scan_locations]
        effect_rates = [loc.effect_rate * 100 for loc in self.controller.scan_locations]

        x_vals = range(len(glitch_rates))

        # Plot all rates
        self.ax_success.fill_between(x_vals, effect_rates, alpha=0.2, color='blue', label='Any Effect')
        self.ax_success.plot(x_vals, glitch_rates, 'g-', linewidth=1.5, label='Glitch (Success)')
        self.ax_success.plot(x_vals, skipped_rates, 'y-', linewidth=1.5, label='Skipped Instructions')
        self.ax_success.plot(x_vals, crash_rates, 'r-', linewidth=1, alpha=0.7, label='Crash')

        self.ax_success.set_xlabel("Location Index", fontsize=8)
        self.ax_success.set_ylabel("Rate (%)", fontsize=8)
        self.ax_success.set_title("EMP Glitch Results by Location", fontsize=10)
        self.ax_success.grid(True, alpha=0.3)
        self.ax_success.set_ylim([0, 100])
        self.ax_success.legend(fontsize=7, loc='upper right')

        self.fig.tight_layout(pad=2.0)
        self.canvas.draw()

        # Update 3D visualization
        self.update_3d_visualization()

    def update_3d_visualization(self):
        """Update the 3D scan visualization"""
        if not self.controller.scan_locations:
            return

        self.ax_3d.clear()

        # Extract coordinates and results
        x_coords = [loc.x for loc in self.controller.scan_locations]
        y_coords = [loc.y for loc in self.controller.scan_locations]
        z_coords = [loc.z for loc in self.controller.scan_locations]
        effect_rates = [loc.effect_rate for loc in self.controller.scan_locations]

        # Color based on effect rate: green = high effect, red = no effect
        colors = []
        for loc in self.controller.scan_locations:
            if loc.glitch_count > 0:
                colors.append('lime')  # Full glitch - bright green
            elif loc.skipped_count > 0:
                colors.append('yellow')  # Skipped - yellow
            elif loc.crash_count > 0:
                colors.append('red')  # Crash - red
            else:
                colors.append('gray')  # Nothing - gray

        # Plot scan path
        self.ax_3d.plot(x_coords, y_coords, z_coords, 'b-', alpha=0.2, linewidth=0.5)

        # Scatter plot with color coding
        scatter = self.ax_3d.scatter(x_coords, y_coords, z_coords,
                                      c=colors, s=30, alpha=0.8, edgecolors='black', linewidth=0.3)

        # Highlight current position if scanning
        if self.controller.scanning:
            self.ax_3d.scatter([self.controller.current_x],
                              [self.controller.current_y],
                              [self.controller.current_z],
                              c='blue', s=150, marker='*', label='Current')

        # Highlight best location if found
        if self.controller.scan_locations:
            best = max(self.controller.scan_locations, key=lambda l: l.effect_rate)
            if best.effect_rate > 0:
                self.ax_3d.scatter([best.x], [best.y], [best.z],
                                  c='magenta', s=200, marker='D',
                                  edgecolors='white', linewidth=2, label='Best')

        self.ax_3d.set_xlabel("X (mm)", fontsize=8)
        self.ax_3d.set_ylabel("Y (mm)", fontsize=8)
        self.ax_3d.set_zlabel("Z (mm)", fontsize=8)
        self.ax_3d.set_title("3D Scan Results\n(Green=Glitch, Yellow=Skipped, Red=Crash, Gray=Nothing)", fontsize=9)
        self.ax_3d.tick_params(labelsize=7)

        # Add legend
        from matplotlib.lines import Line2D
        legend_elements = [
            Line2D([0], [0], marker='o', color='w', markerfacecolor='lime', markersize=8, label='Glitch'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='yellow', markersize=8, label='Skipped'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='red', markersize=8, label='Crash'),
            Line2D([0], [0], marker='o', color='w', markerfacecolor='gray', markersize=8, label='Nothing'),
        ]
        self.ax_3d.legend(handles=legend_elements, loc='upper left', fontsize=7)

        self.fig_3d.tight_layout()
        self.canvas_3d.draw()

    def update_statistics(self):
        self.stats_text.config(state=tk.NORMAL)
        self.stats_text.delete(1.0, tk.END)

        stats = self.controller.get_statistics()

        glitch_rate = stats['glitch_rate'] * 100
        skipped_rate = stats.get('skipped_rate', 0) * 100
        effect_rate = stats.get('effect_rate', 0) * 100
        crash_rate = stats['crash_rate'] * 100

        area_info = ""
        if self.controller.origin_set and self.controller.top_right_set:
            area_info = f"\nScan Area: {self.controller.top_right_x:.2f} x {self.controller.top_right_y:.2f} mm"

        stats_text = f"""
Scan Progress:
--------------
Locations Scanned: {stats['locations_scanned']}
Total EMP Pulses: {stats['total_attempts']}{area_info}

Results:
  Glitches: {stats['total_glitches']}
  Skipped:  {stats.get('total_skipped', 0)}
  Crashes:  {stats['total_crashes']}
  Timeouts: {stats['total_timeouts']}
  Resets:   {stats['total_resets']}
  Nothing:  {stats['total_nothing']}

Rates:
  Glitch Rate:  {glitch_rate:.2f}%
  Skipped Rate: {skipped_rate:.2f}%
  Effect Rate:  {effect_rate:.2f}%
  Crash Rate:   {crash_rate:.2f}%

Recovery:
  Power Cycles: {stats['total_power_cycles']}

Current Position:
  X: {self.controller.current_x:.2f} mm
  Y: {self.controller.current_y:.2f} mm
  Z: {self.controller.current_z:.2f} mm

Status: {'SCANNING' if self.controller.scanning else 'IDLE'}
Target: {self.controller.target_state.name}
        """

        self.stats_text.insert(1.0, stats_text)
        self.stats_text.config(state=tk.DISABLED)

    # ======================== RESULTS DISPLAY ========================

    def show_heatmaps_and_recommendation(self):
        if not self.controller.scan_locations:
            messagebox.showwarning("No Data", "No scan data available")
            return

        # Group data by Z layer
        z_layers = {}
        for loc in self.controller.scan_locations:
            z = round(loc.z, 3)
            if z not in z_layers:
                z_layers[z] = []
            z_layers[z].append(loc)

        heatmap_window = tk.Toplevel(self.root)
        heatmap_window.title("EMFI Scan Results - Heatmaps and Recommendation")
        heatmap_window.geometry("1200x800")

        notebook = ttk.Notebook(heatmap_window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Create recommendation tab
        optimal_location = self.controller.find_optimal_glitch_location()

        if optimal_location:
            rec_frame = ttk.Frame(notebook)
            notebook.add(rec_frame, text="Recommendation")

            rec_text = tk.Text(rec_frame, font=("Courier", 11), wrap=tk.WORD)
            rec_scrollbar = ttk.Scrollbar(rec_frame, orient=tk.VERTICAL, command=rec_text.yview)
            rec_text.configure(yscrollcommand=rec_scrollbar.set)

            rec_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            rec_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)

            stats = self.controller.get_statistics()

            skipped = optimal_location.get('skipped', 0)
            skipped_rate = optimal_location.get('skipped_rate', 0)
            effect_rate = optimal_location.get('effect_rate', 0)

            recommendation = f"""
OPTIMAL GLITCH LOCATION
=======================

Position:
  X: {optimal_location['x']:.3f} mm
  Y: {optimal_location['y']:.3f} mm
  Z: {optimal_location['z']:.3f} mm

Results (from {optimal_location['total']} pulses):
  Glitches:  {optimal_location['glitch']} ({optimal_location['glitch_rate']*100:.1f}%)
  Skipped:   {skipped} ({skipped_rate*100:.1f}%)
  Crashes:   {optimal_location['crash']} ({optimal_location['crash_rate']*100:.1f}%)
  Nothing:   {optimal_location['nothing']} ({optimal_location['nothing']/optimal_location['total']*100:.1f}%)

  Total Effect Rate: {effect_rate*100:.1f}%

Score: {optimal_location['score']:.2f}

---------------------------------------------------

Summary Statistics:
  Scan area: {self.controller.top_right_x:.2f} x {self.controller.top_right_y:.2f} mm
  Total locations scanned: {stats['locations_scanned']}
  Total EMP pulses: {stats['total_attempts']}
  Overall glitch rate: {stats['glitch_rate']*100:.2f}%
  Overall skipped rate: {stats.get('skipped_rate', 0)*100:.2f}%
  Overall effect rate: {stats.get('effect_rate', 0)*100:.2f}%
  Overall crash rate: {stats['crash_rate']*100:.2f}%
  Total power cycles: {stats['total_power_cycles']}

---------------------------------------------------

NEXT STEPS:

1. Move probe to the optimal position shown above
2. Perform additional targeted testing at this location
3. Fine-tune Z-height around {optimal_location['z']:.3f} mm for best results
4. Consider testing neighboring locations for consistency
5. If "skipped" rate is high but "glitch" is low, try adjusting
   EMP timing or probe distance
            """

            rec_text.insert(1.0, recommendation)
            rec_text.config(state=tk.DISABLED)

        # Create Z-layer heatmap tabs
        for z_height in sorted(z_layers.keys()):
            layer_data = z_layers[z_height]

            frame = ttk.Frame(notebook)
            notebook.add(frame, text=f"Z = {z_height:.3f} mm")

            fig = Figure(figsize=(12, 10))

            # Glitch success heatmap (green = good)
            ax1 = fig.add_subplot(221)
            self.plot_heatmap(ax1, layer_data, 'glitch', f'Glitch Success - Z={z_height:.2f}mm', 'Greens')

            # Skipped instructions heatmap (yellow/orange = partial effect)
            ax2 = fig.add_subplot(222)
            self.plot_heatmap(ax2, layer_data, 'skipped', f'Skipped Instructions - Z={z_height:.2f}mm', 'YlOrRd')

            # Combined effect heatmap (glitch + skipped)
            ax3 = fig.add_subplot(223)
            self.plot_heatmap(ax3, layer_data, 'effect', f'Any Effect (Glitch+Skipped) - Z={z_height:.2f}mm', 'Blues')

            # Crash heatmap (red = bad)
            ax4 = fig.add_subplot(224)
            self.plot_heatmap(ax4, layer_data, 'crash', f'Crash Rate - Z={z_height:.2f}mm', 'Reds')

            fig.tight_layout()

            canvas = FigureCanvasTkAgg(fig, frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        notebook.select(0)

        stats = self.controller.get_statistics()
        messagebox.showinfo("EMP Scan Complete",
                           f"EMP scan completed!\n\n"
                           f"Total glitches: {stats['total_glitches']}\n"
                           f"Total skipped: {stats.get('total_skipped', 0)}\n"
                           f"Total crashes: {stats['total_crashes']}\n"
                           f"Glitch rate: {stats['glitch_rate']*100:.2f}%\n"
                           f"Effect rate: {stats.get('effect_rate', 0)*100:.2f}%\n"
                           f"Power cycles: {stats['total_power_cycles']}\n\n"
                           f"Check the heatmaps and recommendation tab!")

    def plot_heatmap(self, ax, layer_data, data_type, title, colormap='RdYlGn'):
        x_coords = [d.x for d in layer_data]
        y_coords = [d.y for d in layer_data]

        x_unique = sorted(set(x_coords))
        y_unique = sorted(set(y_coords))

        grid = np.zeros((len(y_unique), len(x_unique)))

        for d in layer_data:
            x_idx = x_unique.index(d.x)
            y_idx = y_unique.index(d.y)
            if data_type == 'glitch':
                grid[y_idx, x_idx] = d.glitch_rate * 100
            elif data_type == 'skipped':
                grid[y_idx, x_idx] = d.skipped_rate * 100
            elif data_type == 'effect':
                grid[y_idx, x_idx] = d.effect_rate * 100
            elif data_type == 'crash':
                grid[y_idx, x_idx] = d.crash_rate * 100

        im = ax.imshow(grid, cmap=colormap, interpolation='nearest', aspect='auto',
                      extent=[min(x_unique), max(x_unique), min(y_unique), max(y_unique)],
                      origin='lower', vmin=0, vmax=100)

        ax.set_xlabel("X Position (mm)", fontsize=8)
        ax.set_ylabel("Y Position (mm)", fontsize=8)
        ax.set_title(title, fontsize=9)
        ax.tick_params(labelsize=7)

        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Rate (%)', fontsize=8)
        cbar.ax.tick_params(labelsize=7)

        ax.grid(True, alpha=0.3, color='black', linewidth=0.5)

    # ======================== CLEANUP ========================

    def on_closing(self):
        # Set closing flag to stop periodic updates
        self.closing = True

        if self.controller.scanning:
            if not messagebox.askyesno("Scan in Progress",
                                       "Scan is running. Stop and exit?"):
                self.closing = False  # User cancelled, reset flag
                return
            self.controller.stop_scan()

        if self.monitor_running:
            self.stop_serial_monitor()

        if self.board_monitor_running:
            self.stop_board_monitor()

        # Close serial log file
        self.stop_serial_log()

        self.controller.cleanup()
        self.root.destroy()


def main():
    root = tk.Tk()

    style = ttk.Style()
    style.theme_use('clam')

    app = EMFIFaultierGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()
