import tkinter as tk
from tkinter import ttk, messagebox
from functools import partial
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import MEMS_Control_Function
import RPi.GPIO as GPIO
import socket
import time
import BuildCom
import threading
import sys
from queue import Queue, Empty
import numpy as np
from datetime import timedelta
import logging

GPIO.setwarnings(False)
logging.basicConfig(level=logging.INFO)

def main():
    Vbias = 90
    VdiffMax = 55
    Vdiff = 30
    lower_threshold = -9  # Set appropriate value
    upper_threshold = 9  # Set appropriate value
    global ax, canvas, recfig, axr, tracker
    clock_generator = MEMS_Control_Function.ClockSignalGenerator(pin1=22, pin2=27, frequency=30000)
    client_socket = None
    data_queue = Queue()
    
    def connect_to_server():
        nonlocal client_socket
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.settimeout(5)
        success, client_socket = BuildCom.connect_to_RX(client_socket)
        if success:
            print("Connected to server successfully")
            # Start the data receiving thread
            data_thread = threading.Thread(target=BuildCom.receive_coordinates, args=(client_socket, data_queue))
            data_thread.start()
        else:
            print("Failed to connect to the server")

    def start_tracking():
        global tracker
        if client_socket:
            tracker = Tracker(data_queue, Vbias, Vbias, Vdiff, Vdiff)
            tracker.start_tracking()
        else:
            print("Please connect to the server first")

    def close_server():
        if client_socket:
            client_socket.close()
            logging.info("Connection closed successfully")
    
    # Generate GUI Interface
    app = tk.Tk()
    app.title("Hardware Control Interface")
    app.geometry("1300x1080")
    
    close_button = tk.Button(app, text="Close App", command=app.destroy)
    close_button.grid(row=6, column=0, pady=10)

    # MEMS START GUI
    ttk.Button(app, text="Enable Driver", command=MEMS_Control_Function.Enable_Driver).grid(row=0, column=0, padx=10, pady=5)
    ttk.Button(app, text="Disable Driver", command=MEMS_Control_Function.Disable_Driver).grid(row=0, column=1, padx=10, pady=5)
    ttk.Button(app, text="Enable FCLK", command=clock_generator.start).grid(row=1, column=0, padx=10, pady=5)
    ttk.Button(app, text="Disable FCLK", command=clock_generator.stop).grid(row=1, column=1, padx=10, pady=5)

    # Initialize MEMS GUI
    ttk.Button(app, text="Initialize DAC", command=MEMS_Control_Function.initialize_dac).grid(row=2, column=0, padx=10, pady=5)
    ttk.Button(app, text="Reset All Bias", command=partial(MEMS_Control_Function.Reset_AllBias, Vbias)).grid(row=2, column=1, padx=10, pady=5)

    # Vdiff Input GUI
    tk.Label(app, text="Enter Vdiff:").grid(row=3, column=0, padx=10, pady=5)
    vdiff_entry = tk.Entry(app)
    vdiff_entry.grid(row=3, column=1, padx=5, pady=5)
    
    # Rotation MEMS with accord coordination
    tk.Label(app, text="Enter X coordinate (-1 to 1):").grid(row=4, column=0, padx=10, pady=5)
    x_entry = tk.Entry(app)
    x_entry.grid(row=4, column=1, padx=10, pady=5)

    tk.Label(app, text="Enter Y coordinate (-1 to 1):").grid(row=5, column=0, padx=10, pady=5)
    y_entry = tk.Entry(app)
    y_entry.grid(row=5, column=1, padx=10, pady=5)

    # Submit GUI
    submit_button = tk.Button(app, text="Submit", command=partial(submit, x_entry, y_entry, vdiff_entry))
    submit_button.grid(row=6, column=1, padx=10, pady=5)

    # Vdiff Submit GUI
    Vdsubmit_button = tk.Button(app, text="Update Vdiff", command=lambda: Vdiff_input(vdiff_entry.get()))
    Vdsubmit_button.grid(row=7, column=0, padx=10, pady=5)
    
    # Build Communication GUI
    ConServer_button = tk.Button(app, text="Connect server", command=connect_to_server)
    ConServer_button.grid(row=7, column=1, padx=10, pady=5)

    # Close Communication GUI
    CloseServer_button = tk.Button(app, text="Close server", command=close_server)
    CloseServer_button.grid(row=8, column=1, padx=10, pady=5)

    # Transmittion Coordinate location GUI
    tk.Label(app, text="Trans spot Location").grid(row=0, column=2)
    fig = Figure(figsize=(3, 3), dpi=100)
    ax = fig.add_subplot(111)
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.axhline(0, color='black', linewidth=0.5)
    ax.axvline(0, color='black', linewidth=0.5)
    ax.grid(color='gray', linestyle='--', linewidth=0.5)
    canvas = FigureCanvasTkAgg(fig, master=app)
    canvas.draw()
    canvas.get_tk_widget().grid(row=1, column=2, rowspan=7, padx=10, pady=5)


    # Log Textbox with Scrollbar
    log_frame = tk.Frame(app)
    log_frame.grid(row=9, column=2, columnspan=1, pady=10, padx=5)

    scrollbar = tk.Scrollbar(log_frame)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

    log_text = tk.Text(log_frame, wrap=tk.WORD, yscrollcommand=scrollbar.set)
    log_text.pack()

    scrollbar.config(command=log_text.yview)
    
        # Redirect print statements to log_text
    redir = RedirectText(log_text)
    sys.stdout = redir
    
    #Tracking control
    Trackstart_button = tk.Button(app, text="Start Tracking", command=start_tracking)
    Trackstart_button.grid(row=9, column=0, padx=10, pady=5)
    Trackstop_button = tk.Button(app, text="Stop Tracking", command=lambda: tracker.stop_tracking() if tracker else None)
    Trackstop_button.grid(row=9, column=1, padx=10, pady=5)
    
    app.mainloop()

    MEMS_Control_Function.Reset_AllBias
    MEMS_Control_Function.Disable_Driver
    clock_generator.stop
    BuildCom.close_server
    
 
def validate_value(x):
    try:
        x = float(x)
    except ValueError:
        return False
    return -1 <= x <= 1

def validate_Vdiff(x):
    try:
        x = float(x)
    except ValueError:
        return False
    return 0 <= x <= 55

def Vdiff_input(vdiff_entry):
    global tracker,Vdiff
    Vdiff_str = vdiff_entry
    if validate_Vdiff(Vdiff_str):
        tracker.update_vdiff = float(Vdiff_str)
        Vdiff = float(Vdiff_str)
        print(f"Vdiff has updated to {Vdiff}!")
    else:
        messagebox.showerror("Invalid Vdiff", "Please enter Vdiff lower than 55V.")

def submit(x_entry, y_entry, vdiff_entry):
    x_coord = x_entry.get()
    y_coord = y_entry.get()
    Vdiff_str = vdiff_entry.get()
    if validate_value(x_coord) and validate_value(y_coord) and validate_Vdiff(Vdiff_str):
        x = float(x_coord)
        y = float(y_coord)
        Vdiff = float(Vdiff_str)
        process_coordinates(x, y, Vdiff)
        plot_coordinates(x, y)
    else:
        messagebox.showerror("Invalid Input", "Please enter valid coordinates in the range [-1, 1] and a valid Vdiff.")

def process_coordinates(x, y, Vdiff):
    X_V = x * Vdiff
    Y_V = y * Vdiff
    MEMS_Control_Function.Rotation_Control_X(90, X_V, 55)
    MEMS_Control_Function.Rotation_Control_Y(90, Y_V, 55)

def plot_coordinates(x, y):
    global ax, canvas
    # clear plot before
    ax.clear()

    # Plot
    ax.plot(x, y, 'ro')  
    ax.set_xlim(-1, 1)
    ax.set_ylim(-1, 1)
    ax.axhline(0, color='black', linewidth=0.5)
    ax.axvline(0, color='black', linewidth=0.5)
    ax.grid(color='gray', linestyle='--', linewidth=0.5)

    # Update plot
    canvas.draw()

class Tracker(threading.Thread):
    def __init__(self, data_queue, x_bias, y_bias, x_diff, y_diff):
        super().__init__()
        self.data_queue = data_queue
        self.x_bias = x_bias
        self.y_bias = y_bias
        self.x_diff = x_diff
        self.y_diff = y_diff
        self.stop_event = threading.Event()
        self.cycle_complete = threading.Event()
        self.scanning = threading.Event()
        self.new_data_event = threading.Event()
        self.data_lock = threading.Lock()
        self.position_timestamps = {}
        self.reset()

    def run(self):
        if self.stop_event.is_set():
            return

        self.cycle_complete.clear()
        self.scanning.set()
        print("Scanning started.")

        step = 0.1
        x = np.arange(-1, 1 + step, step)
        #x = np.concatenate((x, x[::-1]))
        y_value = 0

        for x_value in x:
            if self.stop_event.is_set():
                print("Scanning stopped.")
                return

            X_diff = x_value * self.x_diff
            Y_diff = y_value * self.y_diff
            MEMS_Control_Function.Rotation_Control_X(self.x_bias, X_diff, 55)
            MEMS_Control_Function.Rotation_Control_Y(self.y_bias, Y_diff, 55)

            # Get data from queue
            try:
                result = self.data_queue.get(timeout=1)
                with self.data_lock:
                    self.timestamp, self.events, self.direction_x, self.direction_y = result
                    print(f"TXport receive_timestamp={self.timestamp}, events={self.events}, direction_x={self.direction_x}, direction_y={self.direction_y}")
            except Empty:
                print("No data received within timeout period.")
                continue

            if self.timestamp is not None:
                self.position_timestamps[(x_value, y_value)] = self.timestamp
                if self.events > self.max_events:
                    self.max_events = self.events
                    self.max_events_coords = (x_value, y_value)
                    self.max_events_diff = (self.x_diff * x_value, self.y_diff * y_value)
                    time.sleep(0.001)

        if self.max_events_coords != (0, 0):
            self.ignore_events = True
            MEMS_Control_Function.Rotation_Control_X(self.x_bias, self.max_events_diff[0], 55)
            MEMS_Control_Function.Rotation_Control_Y(self.y_bias, self.max_events_diff[1], 55)
            print(f"Cycle complete. Max events: {self.max_events} at position {self.max_events_coords}")
            self.scanning_completed = True
            self.cycle_complete.set()
            self.scanning.clear()
            self.ignore_events = False
            print("Scanning cycle complete, stopping scanning.")
        else:
            print("Do not need to move spots")
        self.stop_event.set()

    def start_tracking(self):
        if not self.is_alive():
            self.scanning_completed = False
            self.stop_event.clear()
            self.start()
            print("Tracking started.")
        else:
            print("Tracking is already running")

    def stop_tracking(self):
        self.stop_event.set()
        if self.is_alive():
            self.join()
        self.reset()
        print("Tracking stopped.")

    def reset(self):
        self.max_events = 0
        self.max_events_coords = (0, 0)
        self.max_events_diff = (0, 0)
        self.cycle_complete.clear()
        self.scanning_completed = False
        self.stop_event.clear()
        self.position_timestamps.clear()
                
    # Scroll Text class
class RedirectText(object):
    def __init__(self, widget):
        self.widget = widget

    def write(self, string):
        self.widget.insert(tk.END, string)
        self.widget.see(tk.END)  
    
    def flush(self):
        pass

# Call the main function    
if __name__ == "__main__":
    main()
