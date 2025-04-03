## Sina Allen
## March 29, 2025
## Client UI for controlling 6 dof robotic hand

import tkinter as tk
from tkinter import ttk, messagebox, simpledialog, filedialog
import serial
import time
import struct
import threading
import csv
import os

# Register address dictionary (example, adjust as needed)
regdict = {
    'ID': 2, 'baudrate': 12, 'curLocat': 26, 'zeroCalibra': 31, 'overCurproSet': 32,
    'tarLocatSet': 55, 'fSensorDada': 76, 'fOriginalValue': 78, 'forceAct': 98, 'warmUpSta': 100
}

# Global variables
MIN_POS = 25
MAX_POS = 1775
ser = None
actuator_positions = {id: MIN_POS for id in range(1, 6)}  # Initial target positions
gestures = []  # List to store up to 10 gestures
pos_labels = {}
temp_labels = {}
current_labels = {}
force_labels = {}
gesture_listbox = None
mode_var = None  # Variable to track jogging mode (0: normal, 1: micro)
serial_lock = threading.Lock()  # Lock for serial port access

# Serial port setup
def openSerial(port, baudrate):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.timeout = 1  # Read timeout
    ser.write_timeout = 1  # Write timeout
    ser.open()
    return ser

# Write register (example, adjust as needed)
def writeRegister(ser, id, add, num, val):
    with serial_lock:
        bytes = [0x55, 0xAA, num + 2, id, 0x02, add]
        for i in range(num):
            bytes.append(val[i])
        checksum = sum(bytes[2:]) & 0xFF
        bytes.append(checksum)
        try:
            ser.write(bytes)
            time.sleep(0.01)
            ser.read_all()
        except serial.SerialTimeoutException:
            print(f"Write timeout in writeRegister for actuator {id}")

# Read register (example, adjust as needed)
def readRegister(ser, id, add, num, mute=False):
    with serial_lock:
        bytes = [0x55, 0xAA, num + 2, id, 0x01, add, num]
        checksum = sum(bytes[2:]) & 0xFF
        bytes.append(checksum)
        try:
            ser.write(bytes)
            time.sleep(0.01)
            recv = ser.read_all()
            if len(recv) == 0:
                return []
            num = (recv[2] & 0xFF) - 2
            val = [recv[6 + i] for i in range(num)]
            if not mute:
                print('Read register values:', val)
            return val
        except serial.SerialTimeoutException:
            print(f"Write timeout in readRegister for actuator {id}")
            return []

# Query actuator status
def control(ser, id):
    command = [0x55, 0xAA, 0x03, id, 0x04, 0x00, 0x22]
    checksum = sum(command[2:]) & 0xFF
    command.append(checksum)
    with serial_lock:
        try:
            ser.write(bytes(command))
            time.sleep(0.01)
            response = ser.read(22)
            if len(response) == 22 and response[0] == 0xAA and response[1] == 0x55:
                calc_checksum = sum(response[2:21]) & 0xFF
                if calc_checksum == response[21]:
                    current_pos = struct.unpack('<h', response[9:11])[0]
                    temp = struct.unpack('<b', response[11:12])[0]
                    current = struct.unpack('<H', response[12:14])[0]
                    force = struct.unpack('<h', response[14:16])[0]
                    return current_pos, temp, current, force
        except serial.SerialTimeoutException:
            print(f"Write timeout in control for actuator {id}")
    return None

# Broadcast position command for 5 actuators
def broadcast(ser, num, val1, val2, val3, val4, val5):
    bytes = [0x55, 0xAA, 1 + num * 3, 0xff, 0xf2]
    actuator_values = [val1, val2, val3, val4, val5]
    for i, val in enumerate(actuator_values):
        bytes.append(i + 1)
        bytes.append(val & 0xFF)
        bytes.append((val >> 8) & 0xFF)
    checksum = sum(bytes[2:]) & 0xFF
    bytes.append(checksum)
    with serial_lock:
        try:
            ser.write(bytes)
            time.sleep(0.01)
        except serial.SerialTimeoutException:
            print("Write timeout in broadcast")

# Extend actuator with mode-dependent step size
def extend_actuator(id):
    step = 10 if mode_var.get() else 100  # Micro mode: 10, Normal mode: 100
    actuator_positions[id] = min(actuator_positions[id] + step, MAX_POS)
    broadcast(ser, 5, *[actuator_positions[i] for i in range(1, 6)])

# Retract actuator with mode-dependent step size
def retract_actuator(id):
    step = 10 if mode_var.get() else 100  # Micro mode: 10, Normal mode: 100
    actuator_positions[id] = max(actuator_positions[id] - step, MIN_POS)
    broadcast(ser, 5, *[actuator_positions[i] for i in range(1, 6)])

# Extend all actuators to MAX_POS
def extend_all():
    for id in range(1, 6):
        actuator_positions[id] = MAX_POS
    actuator_positions[5] = MAX_POS - 300  # Adjust actuator 5 to avoid collision
    broadcast(ser, 5, MAX_POS, MAX_POS, MAX_POS, MAX_POS, MAX_POS-300)

# Retract all actuators to MIN_POS
def retract_all():
    for id in range(1, 6):
        actuator_positions[id] = MIN_POS
    broadcast(ser, 5, MIN_POS, MIN_POS, MIN_POS, MIN_POS, MIN_POS)

# Prompt user for gesture name
def get_gesture_name(default_name):
    return simpledialog.askstring("Gesture Name", "Enter a name for this gesture:", initialvalue=default_name)

# Save current positions as a gesture
def save_gesture():
    if len(gestures) < 10:  # Increased max gestures to 10
        default_name = f"Gesture {len(gestures) + 1}"
        gesture_name = get_gesture_name(default_name)
        if gesture_name:  # If the user didn't cancel the dialog
            positions = [actuator_positions[id] for id in range(1, 6)]
            gestures.append({'name': gesture_name, 'positions': positions})
            update_gesture_listbox()
    else:
        messagebox.showinfo("Info", "Maximum of 10 gestures reached.")

# Update the gesture listbox with current gestures
def update_gesture_listbox():
    gesture_listbox.delete(0, tk.END)
    for gesture in gestures:
        gesture_listbox.insert(tk.END, gesture['name'])

# Play selected gesture
def play_gesture():
    selected = gesture_listbox.curselection()
    if selected:
        index = selected[0]
        gesture = gestures[index]
        for id, pos in enumerate(gesture['positions'], start=1):
            actuator_positions[id] = pos
        broadcast(ser, 5, *gesture['positions'])
        threading.Timer(1.5, stop_after_gesture).start()

# Stop all actuators after 1 second (for gestures only)
def stop_after_gesture():
    current_positions = []
    for id in range(1, 6):
        status = control(ser, id)
        if status:
            current_pos, _, _, _ = status
            current_positions.append(current_pos)
        else:
            current_positions.append(actuator_positions[id])
    broadcast(ser, 5, *current_positions)
    for id, pos in enumerate(current_positions, start=1):
        actuator_positions[id] = pos

# Edit selected gesture (positions)
def edit_gesture():
    selected = gesture_listbox.curselection()
    if selected:
        index = selected[0]
        positions = [actuator_positions[id] for id in range(1, 6)]
        gestures[index]['positions'] = positions
        messagebox.showinfo("Info", f"{gestures[index]['name']} updated with current positions.")

# Rename selected gesture
def rename_gesture():
    selected = gesture_listbox.curselection()
    if selected:
        index = selected[0]
        current_name = gestures[index]['name']
        new_name = get_gesture_name(current_name)
        if new_name:  # If the user didn't cancel the dialog
            gestures[index]['name'] = new_name
            update_gesture_listbox()
            # Reselect the renamed gesture
            gesture_listbox.selection_set(index)

# Remove selected gesture
def remove_gesture():
    selected = gesture_listbox.curselection()
    if selected:
        index = selected[0]
        del gestures[index]
        update_gesture_listbox()

# Save gestures to CSV file
def save_gestures_to_file():
    if not gestures:
        messagebox.showinfo("Info", "No gestures to save.")
        return
        
    file_path = filedialog.asksaveasfilename(
        defaultextension=".csv",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        title="Save Gestures"
    )
    
    if not file_path:  # User canceled
        return
        
    try:
        with open(file_path, 'w', newline='') as csv_file:
            writer = csv.writer(csv_file)
            # Write header
            writer.writerow(['name', 'position1', 'position2', 'position3', 'position4', 'position5'])
            # Write gestures
            for gesture in gestures:
                row = [gesture['name']] + gesture['positions']
                writer.writerow(row)
        messagebox.showinfo("Success", f"Gestures saved to {file_path}")
    except Exception as e:
        messagebox.showerror("Error", f"Failed to save gestures: {str(e)}")

# Load gestures from CSV file
def load_gestures_from_file():
    file_path = filedialog.askopenfilename(
        defaultextension=".csv",
        filetypes=[("CSV files", "*.csv"), ("All files", "*.*")],
        title="Load Gestures"
    )
    
    if not file_path:  # User canceled
        return
        
    try:
        loaded_gestures = []
        with open(file_path, 'r', newline='') as csv_file:
            reader = csv.reader(csv_file)
            header = next(reader)  # Skip header row
            
            for row in reader:
                if len(row) >= 6:  # Name + 5 positions
                    name = row[0]
                    positions = [int(pos) for pos in row[1:6]]
                    loaded_gestures.append({'name': name, 'positions': positions})
        
        if loaded_gestures:
            # Check if we'll exceed max gestures
            if len(loaded_gestures) > 10:
                loaded_gestures = loaded_gestures[:10]
                messagebox.showwarning("Warning", "Only the first 10 gestures were loaded (maximum limit).")
            
            # Replace current gestures with loaded ones
            global gestures
            gestures = loaded_gestures
            update_gesture_listbox()
            messagebox.showinfo("Success", f"Loaded {len(loaded_gestures)} gestures from {file_path}")
        else:
            messagebox.showinfo("Info", "No gestures found in the file.")
    except Exception as e:
        messagebox.showerror("Error", f"Failed to load gestures: {str(e)}")

# Update actuator status in GUI with error handling
def update_status():
    while True:
        for id in range(1, 6):
            status = control(ser, id)
            if status:
                current_pos, temp, current, force = status
                pos_labels[id].config(text=f"Position: {current_pos}")
                temp_labels[id].config(text=f"Temp: {temp}°C")
                current_labels[id].config(text=f"Current: {current}mA")
                force_labels[id].config(text=f"Force: {force}")
            else:
                print(f"Failed to get status for actuator {id}")
        time.sleep(0.2)

# Dance sequence function
def dance():
    # Step 1: Extend actuators 1 to 5 one by one
    for id in range(1, 6):
        actuator_positions[id] = MAX_POS  # Fully extend
        broadcast(ser, 5, *[actuator_positions[i] for i in range(1, 6)])
        time.sleep(0.2)  # Wait 0.2 second between each

    # Step 2: Retract actuators 5 to 1 one by one
    for id in range(5, 0, -1):
        actuator_positions[id] = MIN_POS  # Fully retract
        broadcast(ser, 5, *[actuator_positions[i] for i in range(1, 6)])
        time.sleep(0.5)  
    
    for id in range(1, 6):
        actuator_positions[id] = MAX_POS  # Fully extend
        broadcast(ser, 5, *[actuator_positions[i] for i in range(1, 6)])
        time.sleep(0.5)  
    
    # Step 2: Retract actuators 5 to 1 one by one
    for id in range(5, 0, -1):
        actuator_positions[id] = MIN_POS  # Fully retract
        broadcast(ser, 5, *[actuator_positions[i] for i in range(1, 6)])
        time.sleep(0.5)  
    time.sleep(1)  # Wait 1 second between each

    # Step 3: Wiggle fingers (actuators 1-4) for 10 seconds
    start_time = time.time()
    while time.time() - start_time < 10:  # Wiggle for 10 seconds
        for i in range(1, 7):
            broadcast(ser, 5, cycle(i), cycle(i-1), cycle(i-2), cycle(i-3), cycle(i-4))
            time.sleep(0.2)

    # Step 4: Final retraction of all actuators to MIN_POS
    retract_all()
    time.sleep(1)  # Wait for all actuators to retract

    # F yeah
    broadcast(ser, 5, 25,25,225,1775,1775)
    time.sleep(3)  # Wait for all actuators to retract
    
    # Peace sign
    broadcast(ser, 5, 1775,25,225,25,1775)
    time.sleep(3)  # Wait for all actuators to retract

    retract_all()

# Start dance in a separate thread to keep GUI responsive
def start_dance():
    dance_thread = threading.Thread(target=dance)
    dance_thread.start()

def cycle(index):
    vals = [MIN_POS, 500, 1000, 1500, 1000, 500]
    return vals[index % len(vals)]

# Load gestures at startup if file exists
def load_gestures_at_startup():
    default_path = "gestures.csv"
    if os.path.exists(default_path):
        try:
            loaded_gestures = []
            with open(default_path, 'r', newline='') as csv_file:
                reader = csv.reader(csv_file)
                header = next(reader)  # Skip header row
                
                for row in reader:
                    if len(row) >= 6:  # Name + 5 positions
                        name = row[0]
                        positions = [int(pos) for pos in row[1:6]]
                        loaded_gestures.append({'name': name, 'positions': positions})
            
            if loaded_gestures:
                # Limit to 10 gestures
                if len(loaded_gestures) > 10:
                    loaded_gestures = loaded_gestures[:10]
                
                global gestures
                gestures = loaded_gestures
                print(f"Loaded {len(loaded_gestures)} gestures from {default_path}")
                return True
        except Exception as e:
            print(f"Failed to load gestures at startup: {str(e)}")
    return False

# Main GUI setup
def main():
    global ser, gesture_listbox, mode_var
    port = 'COM10'  # Adjust to your serial port
    baudrate = 921600  # Adjust to your baud rate
    
    # Open serial port
    ser = openSerial(port, baudrate)
    if ser is None:
        print("Failed to open serial port")
        return
    
    # Initialize all actuators to position MIN_POS
    broadcast(ser, 5, MIN_POS, MIN_POS, MIN_POS, MIN_POS, MIN_POS)
    time.sleep(1)

    # Setup GUI with modern light theme
    root = tk.Tk()
    root.title("Robotic Hand Client")
    
    style = ttk.Style()
    style.theme_use('clam')
    
    # Define a modern light palette and fonts
    light_bg = "#f5f5f5"
    light_fg = "#333333"
    # Button color palette with modern, clean pastel tones:
    btn_extend = "#4A90E2"    # Blue
    btn_retract = "#3D8B8B"   # Darker teal for better white text
    btn_dance = "#9013FE"     # Violet
    btn_save = "#F5A623"      # Orange
    btn_play = "#7ED321"      # Green for Play Selected
    
    # Set global options for a light theme
    root.configure(bg=light_bg)
    style.configure('.', background=light_bg, foreground=light_fg, font=('Segoe UI', 10))
    style.configure('TFrame', background=light_bg)
    style.configure('TLabel', background=light_bg, foreground=light_fg, anchor='center')
    style.configure('TLabelframe', background=light_bg, foreground=light_fg, font=('Segoe UI', 10, 'bold'))
    style.configure('TLabelframe.Label', background=light_bg, foreground=light_fg, font=('Segoe UI', 10, 'bold'))
    
    # Custom button styles with flat modern look, thin black border, and no focus indicator
    style.configure('TButton',
                    relief='solid',
                    borderwidth=0.5,
                    bordercolor="black",
                    padding=6,
                    focusthickness=0,
                    focuscolor=light_bg)
    style.map('TButton', background=[('active', '#e0e0e0')])
    style.configure('Extend.TButton', background=btn_extend, foreground=light_bg)
    style.configure('Retract.TButton', background=btn_retract, foreground=light_bg)
    style.configure('Dance.TButton', background=btn_dance, foreground=light_bg)
    style.configure('Save.TButton', background=btn_save, foreground=light_bg)
    style.configure('Play.TButton', background=btn_play, foreground=light_bg)
    
    style.configure('TCheckbutton', background=light_bg, foreground=light_fg, font=('Segoe UI', 10))
    
    # Create main frames to organize content
    main_frame = ttk.Frame(root, padding="10")
    main_frame.pack(fill=tk.BOTH, expand=True)
    
    # Center content with actuators and gesture panel
    content_frame = ttk.Frame(main_frame)
    content_frame.pack(fill=tk.BOTH, expand=True)
    
    # Left side: Actuator controls
    actuator_frame = ttk.LabelFrame(content_frame, text="Actuator Controls", padding="10")
    actuator_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
    
    # Add Extend All button at the top of actuator_frame
    extend_all_btn = ttk.Button(actuator_frame, text="Extend All Fully", command=extend_all, style='Extend.TButton', takefocus=0)
    extend_all_btn.pack(fill=tk.X, pady=(0, 10))
    
    # Create actuator control grid with centered status texts
    actuator_panels_frame = ttk.Frame(actuator_frame)
    actuator_panels_frame.pack(fill=tk.BOTH, expand=True)
    
    for id in range(1, 6):
        # Frame for each actuator
        actuator_panel = ttk.LabelFrame(actuator_panels_frame, text=f"Actuator {id}", padding="5")
        actuator_panel.grid(row=0, column=id-1, padx=5, pady=5, sticky="nsew")
        
        # Control buttons
        button_frame = ttk.Frame(actuator_panel)
        button_frame.pack(fill=tk.X, pady=5)
        
        extend_btn = ttk.Button(button_frame, text="Extend", command=lambda i=id: extend_actuator(i), style='Extend.TButton', takefocus=0)
        extend_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
        
        retract_btn = ttk.Button(button_frame, text="Retract", command=lambda i=id: retract_actuator(i), style='Retract.TButton', takefocus=0)
        retract_btn.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=2)
        
        # Status indicators with centered text
        status_frame = ttk.Frame(actuator_panel)
        status_frame.pack(fill=tk.X, pady=5)
        
        pos_labels[id] = ttk.Label(status_frame, text="Position: 0", anchor="center")
        pos_labels[id].pack(fill=tk.X, pady=2)
        
        temp_labels[id] = ttk.Label(status_frame, text="Temp: 0°C", anchor="center")
        temp_labels[id].pack(fill=tk.X, pady=2)
        
        current_labels[id] = ttk.Label(status_frame, text="Current: 0mA", anchor="center")
        current_labels[id].pack(fill=tk.X, pady=2)
        
        force_labels[id] = ttk.Label(status_frame, text="Force: 0", anchor="center")
        force_labels[id].pack(fill=tk.X, pady=2)
    
    # Add Retract All button at the bottom of actuator_frame
    retract_all_btn = ttk.Button(actuator_frame, text="Retract All Fully", command=retract_all, style='Retract.TButton', takefocus=0)
    retract_all_btn.pack(fill=tk.X, pady=(10, 0))
    
    # Create a new frame for Dance and Micro Jogging buttons below the retract button
    mode_frame = ttk.Frame(actuator_frame)
    mode_frame.pack(fill=tk.X, pady=(10, 0))
    
    mode_var = tk.IntVar(value=0)  # 0: normal, 1: micro
    mode_button = ttk.Checkbutton(mode_frame, text="Micro-Jogging", variable=mode_var, takefocus=0)
    mode_button.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=(0,5))
    
    dance_btn = ttk.Button(mode_frame, text="Dance Sequence", command=start_dance, style='Dance.TButton', takefocus=0)
    dance_btn.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=(5,0))
    
    # Create the gesture frame with a fixed width using a container frame
    gesture_container = ttk.Frame(content_frame)
    gesture_container.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(5, 0))
    gesture_container.configure(width=300)
    
    # Right side: Gesture controls inside the container
    gesture_frame = ttk.LabelFrame(gesture_container, text="Grips", padding="10")
    gesture_frame.pack(fill=tk.BOTH, expand=True)
    
    # Gesture management 
    gesture_listbox = tk.Listbox(gesture_frame, height=10, borderwidth=2, relief="groove", 
                                 font=('Segoe UI', 10))
    gesture_listbox.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
    
    # Add a scrollbar to the listbox
    scrollbar = ttk.Scrollbar(gesture_frame, orient="vertical", command=gesture_listbox.yview)
    scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
    gesture_listbox.configure(yscrollcommand=scrollbar.set)
    
    # Organize gesture control buttons in grid layout
    button_frame = ttk.Frame(gesture_frame)
    button_frame.pack(fill=tk.X, padx=5, pady=5)
    
    # First row of gesture buttons
    gesture_btn_frame1 = ttk.Frame(button_frame)
    gesture_btn_frame1.pack(fill=tk.X, pady=2)
    
    save_btn = ttk.Button(gesture_btn_frame1, text="Save Current", command=save_gesture, style='Save.TButton', takefocus=0)
    save_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
    
    play_btn = ttk.Button(gesture_btn_frame1, text="Play Selected", command=play_gesture, style='Play.TButton', takefocus=0)
    play_btn.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=2)
    
    # Second row of gesture buttons
    gesture_btn_frame2 = ttk.Frame(button_frame)
    gesture_btn_frame2.pack(fill=tk.X, pady=2)
    
    edit_btn = ttk.Button(gesture_btn_frame2, text="Edit Selected", command=edit_gesture, takefocus=0)
    edit_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
    
    rename_btn = ttk.Button(gesture_btn_frame2, text="Rename", command=rename_gesture, takefocus=0)
    rename_btn.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=2)
    
    # Remove button
    remove_btn = ttk.Button(button_frame, text="Remove", command=remove_gesture, takefocus=0)
    remove_btn.pack(fill=tk.X, pady=2)
    
    # File management separator and frame
    ttk.Separator(gesture_frame, orient='horizontal').pack(fill=tk.X, pady=5)
    
    file_frame = ttk.Frame(gesture_frame)
    file_frame.pack(fill=tk.X, padx=5, pady=5)
    
    # File operation buttons with icons (if available)
    file_buttons_frame = ttk.Frame(file_frame)
    file_buttons_frame.pack(fill=tk.X)
    
    save_file_btn = ttk.Button(file_buttons_frame, text="Save to File", command=save_gestures_to_file, takefocus=0)
    save_file_btn.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=2)
    
    # Load from File now uses the default style (no custom green)
    load_file_btn = ttk.Button(file_buttons_frame, text="Load from File", command=load_gestures_from_file, takefocus=0)
    load_file_btn.pack(side=tk.RIGHT, fill=tk.X, expand=True, padx=2)
    
    # Add a status bar at the bottom
    status_bar = ttk.Label(main_frame, text="Ready. Use keyboard shortcuts: 1-Retract All, 2-Extend All, 3-Dance", 
                           relief=tk.SUNKEN, anchor=tk.W, padding=(5, 2))
    status_bar.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))

    # Key bindings remain the same
    root.bind_all('1', lambda event: retract_all())
    root.bind_all('2', lambda event: extend_all())
    root.bind_all('3', lambda event: start_dance())

    # Bind keys for extending actuators
    root.bind_all('q', lambda event: extend_actuator(1))
    root.bind_all('w', lambda event: extend_actuator(2))
    root.bind_all('e', lambda event: extend_actuator(3))
    root.bind_all('r', lambda event: extend_actuator(4))
    root.bind_all('t', lambda event: extend_actuator(5))

    # Bind keys for retracting actuators
    root.bind_all('a', lambda event: retract_actuator(1))
    root.bind_all('s', lambda event: retract_actuator(2))
    root.bind_all('d', lambda event: retract_actuator(3))
    root.bind_all('f', lambda event: retract_actuator(4))
    root.bind_all('g', lambda event: retract_actuator(5))

    root.focus_set()

    # Configure actuator panel frame for equal sizing
    actuator_panels_frame.grid_columnconfigure(list(range(5)), weight=1)
    
    # Try to load gestures at startup
    load_gestures_at_startup()
    update_gesture_listbox()

    # Start status update thread
    status_thread = threading.Thread(target=update_status, daemon=True)
    status_thread.start()

    # Set a minimum window size
    root.update()
    root.minsize(root.winfo_width(), root.winfo_height())
    
    # Set window icon if file exists
    try:
        root.iconbitmap("brown-jacket.ico")
    except:
        pass  # No icon file available
        
    # Run GUI
    root.mainloop()

if __name__ == "__main__":
    main()
