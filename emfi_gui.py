import tkinter as tk
from tkinter import ttk, messagebox
import threading
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

from emfi_controller import EMFIController


class EMFIScannerGUI:
    """GUI for EMFI Scanner Control"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("EMFI Scanner Control - Ender 3 S1 Pro")
        self.root.geometry("1600x900")
        
        # Create controller instance
        self.controller = EMFIController()
        
        # Configuration parameters
        self.probe_diameter = tk.DoubleVar(value=2.0)
        self.step_size = tk.DoubleVar(value=1.0)
        self.z_increment = tk.DoubleVar(value=0.1)
        self.max_z_height = tk.DoubleVar(value=0.5)
        self.pulses_per_location = tk.IntVar(value=10)
        
        # Serial port parameters
        self.printer_port = tk.StringVar(value="/dev/ttyUSB0")
        self.printer_baudrate = tk.IntVar(value=115200)
        self.emfi_port = tk.StringVar(value="/dev/ttyUSB1")
        self.emfi_baudrate = tk.IntVar(value=115200)
        self.target_port = tk.StringVar(value="/dev/ttyUSB2")
        self.target_baudrate = tk.IntVar(value=115200)
        
        # Movement step size for manual control
        self.move_step = tk.DoubleVar(value=1.0)
        
        # Scan thread
        self.scan_thread = None
        
        # Build GUI
        self.create_widgets()
        
    def create_widgets(self):
        # Main container with two panes
        main_pane = tk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        main_pane.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Left panel - Controls (split into two columns)
        left_container = ttk.Frame(main_pane)
        main_pane.add(left_container, width=800)
        
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
        self.create_manual_control_section(left_column)
        
        # Build right column sections
        self.create_scan_config_section(right_column)
        self.create_scan_control_section(right_column)
        self.create_status_section(right_column)
        
        # Build right panel - visualization
        self.create_visualization_section(right_frame)
        
    def create_connection_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Serial Connections", padding=8)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        # Printer Connection
        printer_frame = ttk.LabelFrame(frame, text="3D Printer (Required)", padding=5)
        printer_frame.pack(fill=tk.X, pady=3)
        
        ttk.Label(printer_frame, text="Port:", font=("Arial", 8)).grid(row=0, column=0, sticky=tk.W, pady=1)
        ttk.Entry(printer_frame, textvariable=self.printer_port, width=15, font=("Arial", 8)).grid(row=0, column=1, pady=1)
        
        ttk.Label(printer_frame, text="Baud:", font=("Arial", 8)).grid(row=1, column=0, sticky=tk.W, pady=1)
        ttk.Entry(printer_frame, textvariable=self.printer_baudrate, width=15, font=("Arial", 8)).grid(row=1, column=1, pady=1)
        
        self.printer_connect_btn = ttk.Button(printer_frame, text="Connect", command=self.connect_printer)
        self.printer_connect_btn.grid(row=2, column=0, columnspan=2, pady=3)
        
        self.printer_status = ttk.Label(printer_frame, text="Disconnected", foreground="red", font=("Arial", 8))
        self.printer_status.grid(row=3, column=0, columnspan=2)
        
        # EMFI Device Connection
        emfi_frame = ttk.LabelFrame(frame, text="EMFI Device (Optional)", padding=5)
        emfi_frame.pack(fill=tk.X, pady=3)
        
        ttk.Label(emfi_frame, text="Port:", font=("Arial", 8)).grid(row=0, column=0, sticky=tk.W, pady=1)
        ttk.Entry(emfi_frame, textvariable=self.emfi_port, width=15, font=("Arial", 8)).grid(row=0, column=1, pady=1)
        
        ttk.Label(emfi_frame, text="Baud:", font=("Arial", 8)).grid(row=1, column=0, sticky=tk.W, pady=1)
        ttk.Entry(emfi_frame, textvariable=self.emfi_baudrate, width=15, font=("Arial", 8)).grid(row=1, column=1, pady=1)
        
        self.emfi_connect_btn = ttk.Button(emfi_frame, text="Connect", command=self.connect_emfi)
        self.emfi_connect_btn.grid(row=2, column=0, columnspan=2, pady=3)
        
        self.emfi_status = ttk.Label(emfi_frame, text="Not Connected", foreground="gray", font=("Arial", 8))
        self.emfi_status.grid(row=3, column=0, columnspan=2)
        
        # Target Device Connection
        target_frame = ttk.LabelFrame(frame, text="Target Device (Optional)", padding=5)
        target_frame.pack(fill=tk.X, pady=3)
        
        ttk.Label(target_frame, text="Port:", font=("Arial", 8)).grid(row=0, column=0, sticky=tk.W, pady=1)
        ttk.Entry(target_frame, textvariable=self.target_port, width=15, font=("Arial", 8)).grid(row=0, column=1, pady=1)
        
        ttk.Label(target_frame, text="Baud:", font=("Arial", 8)).grid(row=1, column=0, sticky=tk.W, pady=1)
        ttk.Entry(target_frame, textvariable=self.target_baudrate, width=15, font=("Arial", 8)).grid(row=1, column=1, pady=1)
        
        self.target_connect_btn = ttk.Button(target_frame, text="Connect", command=self.connect_target)
        self.target_connect_btn.grid(row=2, column=0, columnspan=2, pady=3)
        
        self.target_status = ttk.Label(target_frame, text="Not Connected", foreground="gray", font=("Arial", 8))
        self.target_status.grid(row=3, column=0, columnspan=2)
        
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
        
        self.origin_status = ttk.Label(corner_frame, text="❌ Origin not set", foreground="red", font=("Arial", 8))
        self.origin_status.pack(pady=1)
        
        self.set_top_right_btn = ttk.Button(corner_frame, 
                                            text="2. Mark Top-Right Corner", 
                                            command=self.set_top_right_corner,
                                            state=tk.DISABLED,
                                            style="Accent.TButton")
        self.set_top_right_btn.pack(fill=tk.X, pady=2)
        
        self.top_right_status = ttk.Label(corner_frame, text="❌ Top-right not set", foreground="red", font=("Arial", 8))
        self.top_right_status.pack(pady=1)
        
        self.area_info = ttk.Label(corner_frame, text="Scan Area: Not configured", 
                                   font=("Arial", 8, "italic"))
        self.area_info.pack(pady=3)
        
    def create_scan_config_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Scan Configuration", padding=8)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        config_items = [
            ("Probe Diameter (mm):", self.probe_diameter),
            ("Step Size (mm):", self.step_size),
            ("Z Increment (mm):", self.z_increment),
            ("Max Z Height (mm):", self.max_z_height),
            ("Pulses per Location:", self.pulses_per_location)
        ]
        
        for i, (label, var) in enumerate(config_items):
            ttk.Label(frame, text=label, font=("Arial", 8)).grid(row=i, column=0, sticky=tk.W, pady=1)
            ttk.Entry(frame, textvariable=var, width=12, font=("Arial", 8)).grid(row=i, column=1, pady=1, padx=3)
        
        self.grid_info = ttk.Label(frame, text="Grid: Configure chip area first", font=("Arial", 8))
        self.grid_info.grid(row=len(config_items), column=0, columnspan=2, pady=3)
        
        ttk.Button(frame, text="Calculate Grid", 
                   command=self.update_grid_info).grid(row=len(config_items)+1, column=0, columnspan=2, pady=2)
        
    def create_scan_control_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Scan Control", padding=8)
        frame.pack(fill=tk.X, padx=5, pady=5)
        
        self.start_scan_btn = ttk.Button(frame, text="Start EMFI Scan", 
                                         command=self.start_scan, 
                                         style="Accent.TButton",
                                         state=tk.DISABLED)
        self.start_scan_btn.pack(fill=tk.X, pady=3)
        
        self.stop_scan_btn = ttk.Button(frame, text="Stop Scan", 
                                        command=self.stop_scan, 
                                        state=tk.DISABLED)
        self.stop_scan_btn.pack(fill=tk.X, pady=3)
        
        self.reset_data_btn = ttk.Button(frame, text="Reset Data", 
                                         command=self.reset_data)
        self.reset_data_btn.pack(fill=tk.X, pady=3)
        
    def create_status_section(self, parent):
        frame = ttk.LabelFrame(parent, text="Scan Statistics", padding=10)
        frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.stats_text = tk.Text(frame, height=10, width=35, state=tk.DISABLED, font=("Courier", 8))
        self.stats_text.pack(fill=tk.BOTH, expand=True)
        
        self.update_statistics()
        
    def create_visualization_section(self, parent):
        self.fig = Figure(figsize=(7, 6))
        
        # Top plot - 3D probe position visualization
        self.ax_3d = self.fig.add_subplot(211, projection='3d')
        self.ax_3d.set_title("3D Probe Position", fontsize=10)
        self.ax_3d.set_xlabel("X (mm)", fontsize=8)
        self.ax_3d.set_ylabel("Y (mm)", fontsize=8)
        self.ax_3d.set_zlabel("Z (mm)", fontsize=8)
        self.ax_3d.tick_params(labelsize=7)
        
        # Bottom plot - Success rate by location
        self.ax_success = self.fig.add_subplot(212)
        self.ax_success.set_title("Glitch Success Rate by Location", fontsize=10)
        self.ax_success.set_xlabel("Location Index", fontsize=8)
        self.ax_success.set_ylabel("Success Rate (%)", fontsize=8)
        self.ax_success.tick_params(labelsize=7)
        self.ax_success.grid(True, alpha=0.3)
        
        self.fig.tight_layout(pad=2.0)
        
        self.canvas = FigureCanvasTkAgg(self.fig, parent)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
    # ======================== CONNECTION HANDLERS ========================
    
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
    
    def connect_emfi(self):
        if not self.controller.emfi_connected:
            success, msg = self.controller.connect_emfi(
                self.emfi_port.get(), 
                self.emfi_baudrate.get()
            )
            if success:
                self.emfi_status.config(text="Connected", foreground="green")
                self.emfi_connect_btn.config(text="Disconnect")
                messagebox.showinfo("Success", msg)
            else:
                messagebox.showerror("Connection Error", msg)
        else:
            success, msg = self.controller.disconnect_emfi()
            self.emfi_status.config(text="Not Connected", foreground="gray")
            self.emfi_connect_btn.config(text="Connect")
    
    def connect_target(self):
        if not self.controller.target_connected:
            success, msg = self.controller.connect_target(
                self.target_port.get(), 
                self.target_baudrate.get()
            )
            if success:
                self.target_status.config(text="Connected", foreground="green")
                self.target_connect_btn.config(text="Disconnect")
                messagebox.showinfo("Success", msg)
            else:
                messagebox.showerror("Connection Error", msg)
        else:
            success, msg = self.controller.disconnect_target()
            self.target_status.config(text="Not Connected", foreground="gray")
            self.target_connect_btn.config(text="Connect")
    
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
                self.origin_status.config(text="✓ Origin set at (0, 0, 0)", foreground="green")
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
                    text=f"✓ Top-right set at ({self.controller.top_right_x:.2f}, {self.controller.top_right_y:.2f})", 
                    foreground="green"
                )
                self.start_scan_btn.config(state=tk.NORMAL)
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
                text=f"Scan Area: {width:.2f} mm × {height:.2f} mm",
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
            total_pulses = grid['total_points'] * self.pulses_per_location.get()
            self.grid_info.config(
                text=f"Grid: {grid['x_steps']} x {grid['y_steps']} x {grid['z_steps']} = {grid['total_points']} locations\n"
                     f"Total EMFI pulses: {total_pulses}"
            )
        else:
            self.grid_info.config(text="Grid: Configure chip area first")
    
    # ======================== SCAN HANDLERS ========================
    
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
        
        if not messagebox.askyesno("Start Scan", 
                                    f"Start EMFI scan?\n\n"
                                    f"Scan area: {self.controller.top_right_x:.2f} × {self.controller.top_right_y:.2f} mm\n"
                                    f"Grid: {grid['x_steps']} x {grid['y_steps']} x {grid['z_steps']}\n"
                                    f"Total locations: {grid['total_points']}\n"
                                    f"Total EMFI pulses: {total_pulses}"):
            return
        
        self.start_scan_btn.config(state=tk.DISABLED)
        self.stop_scan_btn.config(state=tk.NORMAL)
        
        # Start scan in separate thread
        self.scan_thread = threading.Thread(target=self.run_scan_thread, daemon=True)
        self.scan_thread.start()
    
    def run_scan_thread(self):
        """Run scan in background thread"""
        def progress_callback(current, total, x, y, z, data):
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
        
        # Update UI on completion
        self.root.after(0, lambda: self.start_scan_btn.config(state=tk.NORMAL))
        self.root.after(0, lambda: self.stop_scan_btn.config(state=tk.DISABLED))
        
        if success:
            self.root.after(0, self.show_heatmaps_and_recommendation)
        else:
            self.root.after(0, lambda: messagebox.showerror("Scan Error", msg))
    
    def stop_scan(self):
        self.controller.stop_scan()
        self.start_scan_btn.config(state=tk.NORMAL)
        self.stop_scan_btn.config(state=tk.DISABLED)
    
    def reset_data(self):
        if messagebox.askyesno("Reset Data", "Clear all scan data and statistics?"):
            self.controller.reset_data()
            
            # Clear graphs
            self.ax_3d.clear()
            self.ax_3d.set_xlabel("X (mm)", fontsize=8)
            self.ax_3d.set_ylabel("Y (mm)", fontsize=8)
            self.ax_3d.set_zlabel("Z (mm)", fontsize=8)
            self.ax_3d.set_title("3D Probe Position", fontsize=10)
            
            self.ax_success.clear()
            self.ax_success.set_xlabel("Location Index", fontsize=8)
            self.ax_success.set_ylabel("Success Rate (%)", fontsize=8)
            self.ax_success.set_title("Glitch Success Rate by Location", fontsize=10)
            self.ax_success.grid(True, alpha=0.3)
            self.ax_success.set_ylim([0, 100])
            
            self.fig.tight_layout(pad=2.0)
            self.canvas.draw()
            
            self.update_statistics()
    
    # ======================== VISUALIZATION ========================
    
    def update_visualization(self):
        if not self.controller.glitch_data:
            return
        
        self.ax_3d.clear()
        self.ax_success.clear()
        
        x_coords = [d['x'] for d in self.controller.glitch_data]
        y_coords = [d['y'] for d in self.controller.glitch_data]
        z_coords = [d['z'] for d in self.controller.glitch_data]
        success_rates = [d['glitch'] / d['total'] * 100 for d in self.controller.glitch_data]
        
        # 3D plot
        self.ax_3d.plot(x_coords, y_coords, z_coords, 'b-', alpha=0.3, linewidth=0.5, label='Scan Path')
        self.ax_3d.scatter(x_coords, y_coords, z_coords, c='gray', s=10, alpha=0.5)
        
        if self.controller.scanning:
            self.ax_3d.scatter([self.controller.current_x], [self.controller.current_y], [self.controller.current_z], 
                              c='blue', s=100, marker='*', label='Current Position')
        
        self.ax_3d.set_xlabel("X (mm)", fontsize=8)
        self.ax_3d.set_ylabel("Y (mm)", fontsize=8)
        self.ax_3d.set_zlabel("Z (mm)", fontsize=8)
        self.ax_3d.set_title("3D Probe Position", fontsize=10)
        self.ax_3d.legend(fontsize=7)
        
        # Success rate plot
        self.ax_success.plot(range(len(success_rates)), success_rates, 'g-', linewidth=1, label='Glitch Rate')
        self.ax_success.fill_between(range(len(success_rates)), success_rates, alpha=0.3)
        self.ax_success.set_xlabel("Location Index", fontsize=8)
        self.ax_success.set_ylabel("Success Rate (%)", fontsize=8)
        self.ax_success.set_title("Glitch Success Rate by Location", fontsize=10)
        self.ax_success.grid(True, alpha=0.3)
        self.ax_success.set_ylim([0, 100])
        self.ax_success.legend(fontsize=7)
        
        self.fig.tight_layout(pad=2.0)
        self.canvas.draw()
    
    def update_statistics(self):
        self.stats_text.config(state=tk.NORMAL)
        self.stats_text.delete(1.0, tk.END)
        
        success_rate = (self.controller.total_glitches/self.controller.total_attempts*100) if self.controller.total_attempts > 0 else 0
        crash_rate = (self.controller.total_crashes/self.controller.total_attempts*100) if self.controller.total_attempts > 0 else 0
        
        area_info = ""
        if self.controller.origin_set and self.controller.top_right_set:
            area_info = f"\nScan Area: {self.controller.top_right_x:.2f} × {self.controller.top_right_y:.2f} mm"
        
        stats = f"""
Scan Progress:
--------------
Total Locations: {len(self.controller.glitch_data)}
Total EMFI Pulses: {self.controller.total_attempts}{area_info}

Results:
Total Glitches: {self.controller.total_glitches}
Total Crashes: {self.controller.total_crashes}

Glitch Rate: {success_rate:.2f}%
Crash Rate: {crash_rate:.2f}%

Current Position:
X: {self.controller.current_x:.2f} mm
Y: {self.controller.current_y:.2f} mm
Z: {self.controller.current_z:.2f} mm

Status: {'SCANNING' if self.controller.scanning else 'IDLE'}
        """
        
        self.stats_text.insert(1.0, stats)
        self.stats_text.config(state=tk.DISABLED)
    
    # ======================== RESULTS DISPLAY ========================
    
    def show_heatmaps_and_recommendation(self):
        if not self.controller.glitch_data:
            messagebox.showwarning("No Data", "No scan data available")
            return
        
        z_layers = self.controller.get_data_by_z_layer()
        
        heatmap_window = tk.Toplevel(self.root)
        heatmap_window.title("EMFI Scan Results - Heatmaps and Recommendation")
        heatmap_window.geometry("1200x800")
        
        notebook = ttk.Notebook(heatmap_window)
        notebook.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        # Create recommendation tab first
        optimal_location = self.controller.find_optimal_glitch_location()
        
        if optimal_location:
            rec_frame = ttk.Frame(notebook)
            notebook.add(rec_frame, text="★ Recommendation")
            
            rec_text = tk.Text(rec_frame, font=("Courier", 11), wrap=tk.WORD)
            rec_scrollbar = ttk.Scrollbar(rec_frame, orient=tk.VERTICAL, command=rec_text.yview)
            rec_text.configure(yscrollcommand=rec_scrollbar.set)
            
            rec_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
            rec_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)
            
            recommendation = f"""
╔═══════════════════════════════════════════════════════════════╗
║                  OPTIMAL GLITCH LOCATION                      ║
╚═══════════════════════════════════════════════════════════════╝

Position:
  X: {optimal_location['x']:.3f} mm
  Y: {optimal_location['y']:.3f} mm
  Z: {optimal_location['z']:.3f} mm

Results (from {optimal_location['total']} pulses):
  Glitches: {optimal_location['glitch']} ({optimal_location['glitch']/optimal_location['total']*100:.1f}%)
  Crashes:  {optimal_location['crash']} ({optimal_location['crash']/optimal_location['total']*100:.1f}%)
  Nothing:  {optimal_location['nothing']} ({optimal_location['nothing']/optimal_location['total']*100:.1f}%)

Score: {optimal_location['score']:.2f}

═══════════════════════════════════════════════════════════════

Summary Statistics:
  Scan area: {self.controller.top_right_x:.2f} × {self.controller.top_right_y:.2f} mm
  Total locations scanned: {len(self.controller.glitch_data)}
  Total EMFI pulses: {self.controller.total_attempts}
  Overall glitch rate: {self.controller.total_glitches/self.controller.total_attempts*100:.2f}%
  Overall crash rate: {self.controller.total_crashes/self.controller.total_attempts*100:.2f}%

═══════════════════════════════════════════════════════════════

INTERPRETATION:

Glitch Rate: Higher is better - indicates successful fault injection
Crash Rate: Lower is better - crashes may reset the device
Score: Composite metric favoring high glitch rate and low crash rate

NEXT STEPS:

1. Move probe to the optimal position shown above
2. Perform additional targeted testing at this location
3. Fine-tune Z-height around {optimal_location['z']:.3f} mm for best results
4. Consider testing neighboring locations for consistency
            """
            
            rec_text.insert(1.0, recommendation)
            rec_text.config(state=tk.DISABLED)
        
        # Create Z-layer heatmap tabs
        for z_height in sorted(z_layers.keys()):
            layer_data = z_layers[z_height]
            
            frame = ttk.Frame(notebook)
            notebook.add(frame, text=f"Z = {z_height:.3f} mm")
            
            fig = Figure(figsize=(10, 8))
            
            ax1 = fig.add_subplot(211)
            self.plot_heatmap(ax1, layer_data, 'glitch', f'Glitch Success Rate - Z = {z_height:.3f} mm')
            
            ax2 = fig.add_subplot(212)
            self.plot_heatmap(ax2, layer_data, 'crash', f'Crash Rate - Z = {z_height:.3f} mm')
            
            fig.tight_layout()
            
            canvas = FigureCanvasTkAgg(fig, frame)
            canvas.draw()
            canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        notebook.select(0)
        
        messagebox.showinfo("Scan Complete", 
                           f"EMFI scan completed!\n\n"
                           f"Total glitches: {self.controller.total_glitches}\n"
                           f"Total crashes: {self.controller.total_crashes}\n"
                           f"Glitch rate: {self.controller.total_glitches/self.controller.total_attempts*100:.2f}%\n\n"
                           f"Check the heatmaps and recommendation tab!")
    
    def plot_heatmap(self, ax, layer_data, data_type, title):
        x_coords = [d['x'] for d in layer_data]
        y_coords = [d['y'] for d in layer_data]
        
        x_unique = sorted(set(x_coords))
        y_unique = sorted(set(y_coords))
        
        grid = np.zeros((len(y_unique), len(x_unique)))
        
        for d in layer_data:
            x_idx = x_unique.index(d['x'])
            y_idx = y_unique.index(d['y'])
            grid[y_idx, x_idx] = d[data_type] / d['total'] * 100
        
        im = ax.imshow(grid, cmap='RdYlGn', interpolation='nearest', aspect='auto',
                      extent=[min(x_unique), max(x_unique), min(y_unique), max(y_unique)],
                      origin='lower', vmin=0, vmax=100)
        
        ax.set_xlabel("X Position (mm)")
        ax.set_ylabel("Y Position (mm)")
        ax.set_title(title)
        
        cbar = plt.colorbar(im, ax=ax)
        cbar.set_label('Success Rate (%)')
        
        ax.grid(True, alpha=0.3, color='black', linewidth=0.5)
    
    # ======================== CLEANUP ========================
    
    def on_closing(self):
        if self.controller.scanning:
            if not messagebox.askyesno("Scan in Progress", 
                                       "Scan is running. Stop and exit?"):
                return
            self.controller.stop_scan()
        
        self.controller.cleanup()
        self.root.destroy()


def main():
    root = tk.Tk()
    
    style = ttk.Style()
    style.theme_use('clam')
    
    app = EMFIScannerGUI(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()


if __name__ == "__main__":
    main()