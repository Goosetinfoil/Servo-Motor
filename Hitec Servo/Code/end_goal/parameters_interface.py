import json
import os
import time
import tkinter as tk
from tkinter import messagebox

import serial

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0

PROFILES_FILE = os.path.join(os.path.dirname(__file__), "servo_profiles.json")

# Editable settings -> register address. Written on profile load.
WRITE_REGISTERS = {
    "speed": 0x54,       # REG_VELOCITY_MAX,        0 .. 4095
    "pos_max": 0xB0,     # REG_POSITION_MAX_LIMIT, -16384 .. 16384
    "pos_min": 0xB2,     # REG_POSITION_MIN_LIMIT, -16384 .. 16384
}

# Read-only IDs -> register address. Read and displayed on button click.
ID_REGISTERS = {
    "ID1 (Actuator ID)": 0x32,
    "ID2 (CAN Node ID)": 0x3E,
}

REG_CONFIG_SAVE = 0x70

# Starting profiles if servo_profiles.json doesn't exist yet.
# Values are raw register units. These are the manual's defaults for
# the position limits; speed left modest. Tune to taste in the app.
DEFAULT_PROFILES = {
    "bottom": {"speed": 1000, "pos_max": 10922, "pos_min": 5462},
    "top": {"speed": 1000, "pos_max": 10922, "pos_min": 5462},
}

REPLY_MARKER = bytes([0x04, 0x15, 0xf3, 0x03, 0x88, 0x0b])


# ---------------------------------------------------------------------
# JSON storage
# ---------------------------------------------------------------------

def load_profiles() -> dict:
    if not os.path.exists(PROFILES_FILE):
        save_profiles(DEFAULT_PROFILES)
        return dict(DEFAULT_PROFILES)
    with open(PROFILES_FILE, "r") as f:
        return json.load(f)


def save_profiles(profiles: dict):
    with open(PROFILES_FILE, "w") as f:
        json.dump(profiles, f, indent=2)


# ---------------------------------------------------------------------
# Serial / DPC-CAN
# ---------------------------------------------------------------------

def initialize(ser: serial.Serial):
    for _ in range(5):
        ser.write(bytes([0x02, 0x53, 0x00, 0xff, 0x52, 0x03, 0x03]))
        time.sleep(0.05)
    for _ in range(11):
        ser.write(bytes([0x02, 0x58, 0x58, 0x03]))
        time.sleep(0.05)
    ser.write(bytes([0x3a, 0x41, 0x3a, 0x41, 0x3a, 0x41]))
    time.sleep(0.05)
    ser.write(bytes([0x02, 0x56, 0x53, 0x00, 0x00, 0x00, 0x00, 0x03]))
    time.sleep(0.2)
    ser.reset_input_buffer()
    for n in range(4):
        frame = bytes([0x02, 0x53, n, 0x01, (n + 0x54) & 0xFF, 0x03, 0x03])
        for _ in range(5):
            ser.write(frame)
            time.sleep(0.05)
    time.sleep(0.5)
    ser.reset_input_buffer()


# --- write ---

def build_write_request(address: int, value: int, seq: int) -> bytes:
    u16 = value & 0xFFFF
    lo, hi = u16 & 0xFF, (u16 >> 8) & 0xFF
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00]
    body = [address & 0xFF, lo, hi, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    return bytes(header + body + [checksum, 0x0a, 0x03])


def write_register(ser: serial.Serial, address: int, value: int, seq: int = 0xd2):
    ser.write(build_write_request(address, value, seq))
    time.sleep(0.05)


# --- read ---

def build_read_request(register: int, seq: int) -> bytes:
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
    body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    return bytes(header + body + [checksum, 0x0a, 0x03])


def find_reply(response: bytes, register: int):
    search_from = 0
    while True:
        idx = response.find(REPLY_MARKER, search_from)
        if idx == -1 or idx + 9 > len(response):
            return None
        if response[idx + 6] == register:
            lo, hi = response[idx + 7], response[idx + 8]
            return lo | (hi << 8)
        search_from = idx + 1


def read_register(ser: serial.Serial, register: int, seq: int,
                  attempts: int = 5, listen_time: float = 0.6):
    frame = build_read_request(register, seq)
    ser.reset_input_buffer()
    collected = b""
    for _ in range(attempts):
        ser.write(frame)
        start = time.time()
        collected = b""
        while time.time() - start < listen_time:
            chunk = ser.read(256)
            if chunk:
                collected += chunk
        value = find_reply(collected, register)
        if value is not None:
            return value
    return None


def to_signed16(value: int) -> int:
    return value - 0x10000 if value >= 0x8000 else value


# ---------------------------------------------------------------------
# Servo-button action: read IDs, write the profile, save.
# ---------------------------------------------------------------------

def load_profile_to_servo(name, profiles, status_var, id_var):
    profile = profiles.get(name)
    if profile is None:
        status_var.set(f"No saved profile for '{name}'.")
        return

    status_var.set(f"Connecting for '{name}'...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        time.sleep(0.2)
        initialize(ser)
    except serial.SerialException as e:
        status_var.set(f"Could not open {PORT}: {e}")
        return

    # 1. Read and display the IDs (read-only, just for confirmation).
    seq = 0xe0
    id_lines = []
    for label, address in ID_REGISTERS.items():
        val = read_register(ser, address, seq)
        seq = (seq + 1) & 0xFF
        id_lines.append(f"{label}: {val if val is not None else '??'}")
    id_var.set("   ".join(id_lines))

    # 2. Write the editable settings (raw values straight through).
    results = []
    for key, address in WRITE_REGISTERS.items():
        if key not in profile:
            continue
        try:
            write_register(ser, address, int(profile[key]), seq=seq)
            results.append(f"{key} OK")
        except serial.SerialException as e:
            results.append(f"{key} FAILED ({e})")
        seq = (seq + 1) & 0xFF

    # 3. Save so it persists after power-off.
    try:
        write_register(ser, REG_CONFIG_SAVE, 0xFFFF, seq=seq)
        results.append("saved")
    except serial.SerialException as e:
        results.append(f"save FAILED ({e})")

    ser.close()
    status_var.set(f"'{name}': " + " | ".join(results))
    print(f"\nLoad profile '{name}': {results}")


# ---------------------------------------------------------------------
# GUI
# ---------------------------------------------------------------------

def build_gui():
    profiles = load_profiles()

    root = tk.Tk()
    root.title("Servo Profile Manager")

    status_var = tk.StringVar(value="Ready.")
    id_var = tk.StringVar(value="IDs: (click a servo to read)")

    # --- Section 1: servo buttons ---
    tk.Label(root, text="Select connected servo:",
             font=("Segoe UI", 10, "bold")).pack(pady=(10, 2))

    button_frame = tk.Frame(root)
    button_frame.pack(pady=5)
    for name in profiles:
        tk.Button(
            button_frame, text=name, width=12,
            command=lambda n=name: load_profile_to_servo(n, profiles, status_var, id_var),
        ).pack(side=tk.LEFT, padx=4)

    # read-only ID display
    tk.Label(root, textvariable=id_var, fg="darkgreen",
             font=("Segoe UI", 9)).pack(pady=(4, 2))

    # status line (wraps instead of overflowing)
    tk.Label(root, textvariable=status_var, fg="blue",
             wraplength=460, justify="left").pack(pady=(2, 10), padx=10, fill="x")

    # --- Section 2: edit a profile ---
    tk.Label(root, text="Edit a profile (raw register values):",
             font=("Segoe UI", 10, "bold")).pack(pady=(10, 2))

    edit_frame = tk.Frame(root)
    edit_frame.pack(pady=5)

    selected_servo = tk.StringVar(value=list(profiles.keys())[0])
    entries = {}

    # friendly labels + valid ranges shown next to each field
    FIELD_INFO = {
        "speed": "Speed (0-4095)",
        "pos_max": "Pos Max (-16384..16384)",
        "pos_min": "Pos Min (-16384..16384)",
    }

    def render_fields(*_):
        for widget in edit_frame.winfo_children():
            widget.destroy()
        entries.clear()
        name = selected_servo.get()
        profile = profiles.get(name, {})
        for row, key in enumerate(WRITE_REGISTERS):  # fixed order
            label = FIELD_INFO.get(key, key)
            tk.Label(edit_frame, text=label).grid(row=row, column=0, sticky="w", padx=4, pady=2)
            entry = tk.Entry(edit_frame, width=12)
            entry.insert(0, str(profile.get(key, 0)))
            entry.grid(row=row, column=1, padx=4, pady=2)
            entries[key] = entry

    def save_edits():
        name = selected_servo.get()
        profile = profiles.setdefault(name, {})
        for key, entry in entries.items():
            raw = entry.get().strip()
            try:
                profile[key] = int(raw)
            except ValueError:
                messagebox.showerror("Invalid value",
                                     f"'{raw}' for {key} isn't a whole number.")
                return
        save_profiles(profiles)
        status_var.set(f"Saved changes to '{name}' profile.")

    dropdown = tk.OptionMenu(root, selected_servo, *profiles.keys(), command=render_fields)
    dropdown.pack()
    render_fields()

    tk.Button(root, text="Save changes", command=save_edits).pack(pady=(6, 10))

    root.mainloop()


if __name__ == "__main__":
    build_gui()