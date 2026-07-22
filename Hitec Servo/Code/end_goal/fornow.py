"""
This program is a tkinter GUI for configuring two different servo/motor controller 
bypassing both the offical configuration app

Tab 1(Hitec Servo) - talks to the Hitec MDB961WP-CAN servo over a DPC-CAN USB adapter
using a reversed-engineered, checksummed 14-byte serial protocol. It supports reading live 
register values, editing a 12-register settings block, and saving/loading named "profiles" so 
a given servo's configurations can be reapplied later. 

Tab 2(CubeMars Servo) - talks to the CubeMars AK80-8 actuator over its R-Link UART
bridge using the VESC packet protocol. Supports reading/writing controller ID, CAN 
baud rate, and a handful of motor-config(MCCONF) fields such as current limits, ERPM 
limits, and speed/position PID gains 
"""

import json
import math
import os
import sys
import time
import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import struct

# ------------------------------------------------
#  Functions and Constants
# ------------------------------------------------

# Constants to find specific adapter chip (Silicon Labs CP210x)
TARGET_VID = 0x10C4 
TARGET_PID = 0xEA60

# Speed of USB connection between computor and the DPC-CAN adapter.
BAUD = 115200  

# Looking for which port the servo is connected to.
def find_hitec_port():
    """Scans all the serial devices connected for one matching the DPC-CAN
    adapter. It return the port name if found. If not found retuns None. 
    """
    # If none, it doesn't crash the program but returns None and an error message.
    for p in serial.tools.list_ports.comports():
        if p.vid == TARGET_VID and p.pid == TARGET_PID:
            return p.device
    return None


# Initialize (wake-up + Auto Scan). This whole block is a byte-for-byte replay of what the official Hitec Configure App sends on startup.
def wake_servo(ser):
    """Brings the servo out of its sleep state so it will respond to register 
    reads and writes. Writes a fixed, timed sequence of bytes to it through a 
    open serial connection. Returns no value. This must be called after opening 
    opening the port and before reading or writing values."""

    seq = 0xc0

    for _ in range(5):
        ser.write(bytes([0x02, 0x53, 0x00, 0xff, 0x52, 0x03, 0x03])) # Sends these bytes to the servo
        time.sleep(0.03) 
    for _ in range(11): # Same as above loop, but with different bytes.
        ser.write(bytes([0x02, 0x58, 0x58, 0x03]))
        time.sleep(0.03)
    ser.write(bytes([0x3a, 0x41, 0x3a, 0x41, 0x3a, 0x41])) # This is a message to the DPC-CAN and not the servo
    time.sleep(0.05)
    ser.write(bytes([0x02, 0x56, 0x53, 0x00, 0x00, 0x00, 0x00, 0x03]))
    time.sleep(0.15)
    ser.reset_input_buffer()

    for n in range(4):
        activate = bytes([0x02, 0x53, n, 0x01, (n + 0x54) & 0xFF, 0x03, 0x03])
        for _ in range(5):
            ser.write(activate)
            time.sleep(0.03)
        # Format A - Message 1 to be sent
        for register in (0xfc, 0xfe, 0x74): # specific servo registers that the app check for(properly version control)
            for _ in range(2): # Happens twice(seq = 192 and seq = 193)
                body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
                hdr = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
                chk = sum(hdr[1:] + body) & 0xFF # Checksum(chk) is a way of checking if the data received matches the transmitted data
                ser.write(bytes(hdr + body + [chk, 0x0a, 0x03]))
                seq = (seq + 1) & 0xFF
                time.sleep(0.02)
            # Format B - Message 2 to be sent
            tailB = {0xfc: 0x50, 0xfe: 0x54, 0x74: 0x40}[register]
            ser.write(bytes([0x02, 0x42, 0x00, 0x00, 0x00, 0x80, 0x96, 0x00, register & 0xFF, 0x00, register & 0xFF, tailB, 0x0a, 0x03]))
            time.sleep(0.02)
            # Format C - Message 3 to be sent
            tailC = {0xfc: [0xcf, 0x08, 0x03], 0xfe: [0xd3, 0x08, 0x03], 0x74: [0xbf, 0x08, 0x03]}[register]
            ser.write(bytes([0x02, 0x41, 0x00, 0x00, 0x96, 0x00, register & 0xFF, 0x00, register & 0xFF] + tailC))
            time.sleep(0.02)

    time.sleep(0.3)
    ser.reset_input_buffer() # Clears any leftover bytes
    "At the end of this block the servo is awake and ready to respond to register reads/writes."

marker = bytes([0xf3, 0x03, 0x88]) # This is the unique byte sequence that appears in every response from the servo
header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80] # This is the unique byte sequence that appears at the start of every read request to the servo

def read_register(ser, register, frame_seq, timeout=3.0): # Same concept as Format A
    """This function requests one register value from the servo and waits 
    for tis reply. `ser` must already be open and woken via wake_servo(); 
    `register` is the register address to read; `frame_seq` is the sequence 
    byte to stamp the request with (must be in the 0xd2-0xd7 range
    Returns the register's 16-bit unsigned value, or None if no valid reply 
    arrived within `timeout` seconds."""

    body = [register & 0xFF, 0x00, 0x00, frame_seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    frame = bytes(header + body + [checksum, 0x0a, 0x03])

    ser.reset_input_buffer() # Clears any leftover bytes
    ser.write(frame) # Writes to the servo

    # Collects the response from the servo within 3 seconds and just stores it instead of looking through it byte by byte.
    collected = b""
    t0 = time.time()
    value = None
    while time.time() - t0 < timeout:
        chunk = ser.read(256) # Reads upto 256 bytes
        if chunk:
            collected += chunk
            idx = collected.find(marker) # After storing the response, it looks for the marker in the response and stores it as idx
            while idx != -1 and idx + 7 <= len(collected): # if the marker is found, and there are more than 7 bytes after the marker, it protects it
                if collected[idx + 4] == register: # This is the register byte inside the reply. It compares this value to the register value from earlier
                    lo, hi = collected[idx + 5], collected[idx + 6]
                    value = lo | (hi << 8)
                    break
                idx = collected.find(marker, idx + 1) # If it doesn't match, it looks for the next marker in the response and repeats the process.
            if value is not None:
                break
    return value


# Function to have '-' in the boxes to write values
def set_entry(entry, value, placeholder="-"):
    """The contents of a tkinter box to write values default placeholder
    is a '-' if the value is None."""
    entry.delete(0, tk.END)
    entry.insert(0, str(value) if value is not None else placeholder)


def to_signed16(v):
    """Converts raw 16-bit unsigned register values to signed values. 
    The servo transmits everything as a usigned register value because 
    some registers(like position), can be negative values"""
    if v is None:
        return None
    return v - 0x10000 if v >= 0x8000 else v


# Links the app to save_profiles.json so that it can save and load the servo profiles.
if getattr(sys, "frozen", False):  
    APP_DIR = os.path.dirname(os.path.abspath(sys.executable))
else:
    APP_DIR = os.path.dirname(os.path.abspath(__file__))

PROFILES_FILE = os.path.join(APP_DIR, "servo_profiles.json")


# When a new profile is created, these are the default values that will be saved to profile.
DEFAULT_PROFILE_SETTINGS = {
    "speed": 1000,
    "pos_max": 10922,
    "pos_min": 5462,
    "pos_mid": 8192,
    "stream_time": 1000,
    "speed_up": 0,
    "speed_dn": 0,
    "sample_point": 1,
    "baud_rate": 0,
    "stream_mode": 0,
    "id1": 1,
    "id2": 0,
}


POSITION_COORDS = {   
    "safety_tether": (389, 636),   # placeholder - actually computed from
    "emergency_vent": (326, 681),  # TETHER_ANGLE_DEG / VENT_ANGLE_DEG at runtime
}


# -------------------------------------------------
#  Hitec Servo 
# -------------------------------------------------


# Spot on the vehicle diagram a profile name refers to by looking for keywords in the name. Returns key into POSITION_COORDS
def match_profile_to_position(profile_name):
    """Determines which spot on the vehicle diagram a profile belongs to
    purely from keywords in its name. Returns a key into POSITION_COORDS """
    name = profile_name.lower()

    if "tether" in name:
        return "safety_tether"
    if "vent" in name:
        return "emergency_vent"

    vertical = "top" if "top" in name else ("bottom" if "bottom" in name else None)
    horizontal = "left" if "left" in name else ("right" if "right" in name else None)
    depth = "inner" if "inner" in name else ("outer" if "outer" in name else None)

    if vertical and horizontal and depth:
        key = f"{vertical}_{horizontal}_{depth}"
        if key in POSITION_COORDS:
            return key
    return None


# This loads up the .json file in the dropdown menu. If the file doesn't exist, it returns an empty dict. 
def load_profiles():
    """Reads servo_profiles.json from disk and returns it as a dict of
    {profile_name: settings_dict}. If the file doesn't exist yet (e.g.
    first run), returns an empty dict instead of raising an error."""
    if not os.path.exists(PROFILES_FILE):
        return {}
    with open(PROFILES_FILE, "r") as f:
        return json.load(f)


# This converts the dict to JSON text and writes it in a readable form if opened directly
def save_profiles(profiles):
    """Write the profiles dict back to disk."""
    with open(PROFILES_FILE, "w") as f:
        json.dump(profiles, f, indent=2)


# Reads ID1, ID2, Spped, Pos Max, Pos Min from the servo 
def read_all_data():
    """Opens the port, wakes the servo, reads all 12 known registers,
    then closes the port. Position values are converted from raw
    unsigned to signed via to_signed16(); everything else is left raw.
    Returns (data_dict, None) on success, or (None, error_message) if
    the adapter wasn't found."""
    port = find_hitec_port()
    if port is None: 
        return None, "DPC-CAN not found. Plug it in"

    # This opens the port everytime ser is called
    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.2)
 
    wake_servo(ser)

    # Reads the following values from the servo and stores them in variables. 
    id1 = read_register(ser, 0x32, 0xd2)
    id2 = read_register(ser, 0x3E, 0xd3)
    speed = read_register(ser, 0x54, 0xd4)
    pos_max = read_register(ser, 0xB0, 0xd5)
    pos_min = read_register(ser, 0xB2, 0xd6)
    pos_mid = read_register(ser, 0xC2, 0xd7)
    baud_rate = read_register(ser, 0x38, 0xd8)
    sample_point = read_register(ser, 0x40, 0xd9)
    stream_time = read_register(ser, 0x2E, 0xda)
    stream_mode = read_register(ser,0x30, 0xdb)
    speed_up = read_register(ser, 0xDC, 0xdc)
    speed_dn = read_register(ser, 0xDE, 0xdd)

    ser.close() # Closes the com port 

    data = { # A dictionary literal where each "key" is a text label matching the same key names throughout the code and the app.
        # Each value is one of the numbers 'read_one' already fetched from the earlier function
        "id1": id1,
        "id2": id2,
        "speed": speed,
        "pos_max": to_signed16(pos_max),
        "pos_min": to_signed16(pos_min),
        "pos_mid": to_signed16(pos_mid),
        "baud_rate": baud_rate,
        "sample_point": sample_point,
        "stream_time": stream_time,
        "stream_mode": stream_mode,
        "speed_up": speed_up,
        "speed_dn": speed_dn,
    }
    return data, None


# This whole block is a faster version of read_all_data() since it only reads ID1 and ID2
def read_ids_only(): 
    """Same open/wake/close cycle as read_all_data(), but only reads
    ID1 and ID2 - used by the 'Read ID' button, which needs to be quick
    and doesn't care about the other 10 registers. Returns
    ((id1, id2), None) on success, or (None, error_message) on failure."""
    # This is a exact copy of waking up the servo, looking for the bytes that represent ID1 and ID2 and storing the values. 
    port = find_hitec_port()
    if port is None:
        return None, "DPC-CAN not found. Plug it in."

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.2)

    wake_servo(ser)

    id1 = read_register(ser, 0x32, 0xd2)
    id2 = read_register(ser, 0x3E, 0xd3)
    ser.close()
    return (id1, id2), None


# This saves the values to the servo 
def write_all(values):
    """Open the servo once, initialize once, write EVERY (register, value)
    pair in `values` (a dict of register -> value), send the save command
    once at the end, then read each register back to confirm.

    Returns (confirmed_dict, error_message_or_None) where confirmed_dict
    maps register -> the value the servo reports after saving (or None if
    that register's read-back failed)."""
    
    port = find_hitec_port()
    if port is None:
        return None, "DPC-CAN not found. Plug it in."

    ser = serial.Serial(port, BAUD, timeout=0.1) # 
    time.sleep(0.2)

    wake_servo(ser)

    # --- write every register (confirmed write frame format), one after
    #     another, with a rolling sequence byte in the proven range ---

    wseq = 0xd4
    for register, value in values.items(): # values.items yields the register and value from the dict earlier to be upacked in the loop     
        u16 = value & 0xFFFF # Clamps value to 16 bits and converts a negative Python number into its correct unsigned wire representation. 
        lo, hi = u16 & 0xFF, (u16 >> 8) & 0xFF
            # Splits u16 into its low byte (lo — bottom 8 bits, no shift needed) and high byte (hi — shift the top 8 bits right by 8 positions, then mask)
        whdr = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00] # Same as read header except the last byte, 0x00 means "I am telling something"
        wbody = [register & 0xFF, lo, hi, wseq & 0xFF] # Same as read body except middle bytes. We are telling something so we place lo, hi
        wchk = sum(whdr[1:] + wbody) & 0xFF # Exact same checksum 
        ser.write(bytes(whdr + wbody + [wchk, 0x0a, 0x03])) # Exact same write 
        wseq = (wseq + 1) & 0xFF # Adding 1 to the d4 so loop repeates for d5 and so on
        time.sleep(0.15)

    # This code below basically tell the servo to save everything into its memory permanently
    shdr = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00]
    sbody = [0x70, 0xff, 0xff, wseq & 0xFF] # writing the magic value 0xffff into register 0x70 (the designated "save" register) tells the servo to commit
    schk = sum(shdr[1:] + sbody) & 0xFF
    ser.write(bytes(shdr + sbody + [schk, 0x0a, 0x03]))
    wseq = (wseq + 1) & 0xFF
    time.sleep(0.3)

    # --- let the servo/DPC-CAN catch up from the burst of writes before
    #     trying to read anything back. ACTIVELY read and discard for a
    #     bit (don't sleep blind) - a blind sleep here stalls the USB
    #     pipe and makes the backlog worse, not better (we proved this
    #     with the read-side backlog issue earlier). ---
    settle_until = time.time() + 1.5
    while time.time() < settle_until:
        ser.read(256)
    ser.reset_input_buffer()

    # --- read each register back to confirm what actually stuck ---
    # IMPORTANT: use a FRESH sequence counter back in the proven-good
    # range here, rather than continuing to count up from the writes.
    # We found earlier (the ID1/ID2 bug) that brand-new/unused sequence
    # numbers get silently ignored, while numbers reused from the same
    # neighborhood as the wake-up sequence's own counter reliably work.
    # By the time the writes+save are done, wseq has drifted well past
    # that safe range - so we reset it here instead of reusing it.
    rseq = 0xd2
    confirmed = {}
    for register in values:
        value_back = read_register(ser, register, rseq, timeout=1.5)
        rseq = (rseq + 1) & 0xFF
        value_back = to_signed16(value_back)
        confirmed[register] = value_back

    ser.close()
    return confirmed, None

# ---------------------------------------------------------------------
# CubeMars AK80-8 (VESC-protocol, R-Link/CH340 adapter)
# ---------------------------------------------------------------------

CUBEMARS_TARGET_VID = 0x1A86
CUBEMARS_TARGET_PID = 0x7523
CUBEMARS_BAUD = 921600

CUBEMARS_CAN_BAUD_LABELS = {
    0: "125K", 1: "250K", 2: "500K", 3: "1M",
}

CUBEMARS_MCCONF_OFFSETS = {
    "motor_max": 9,
    "motor_min": 13,
    "batt_max": 17,
    "batt_min": 21,
    "min_erpm": 29,
    "max_erpm": 33,
    "speed_kp": 324,
    "speed_ki": 328,
    "position_kp": 345,
    "position_ki": 349,
    "position_kd": 353,
}


CUBEMARS_DISPLAY_NAMES = {
    "motor_max": "Motor Max(Amps)",
    "motor_min": "Motor Min(Amps)",
    "batt_max": "Batt Max(Amps)",
    "batt_min": "Batt Min(Amps)",
    "min_erpm": "Min ERPM",
    "max_erpm": "Max ERPM",
    "speed_kp": "Speed KP",
    "speed_ki": "Speed KI",
    "position_kp": "Position KP",
    "position_ki": "Position KI",
    "position_kd": "Position KD",
}


def cubemars_find_port():
    """Same idea as find_hitec_port(), but for the CH340/R-Link adapter's
    VID/PID. Returns the port name if the CubeMars adapter is plugged in,
    otherwise None."""
    for p in serial.tools.list_ports.comports():
        if p.vid == CUBEMARS_TARGET_VID and p.pid == CUBEMARS_TARGET_PID:
            return p.device
    return None


def cubemars_crc16(data: bytes) -> int:
    """Computes the CRC16-XMODEM checksum of `data`, as required by the
    VESC packet protocol. This is a standard bit-by-bit CRC: for each
    byte, XOR it into the top of a 16-bit running value, then shift left
    8 times, XORing in the polynomial 0x1021 whenever the top bit is set.
    Returns the final 16-bit checksum."""
    crc = 0
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc = crc << 1
            crc = crc & 0xFFFF
    return crc


def cubemars_build_packet(payload: bytes) -> bytes:
    """Wraps a raw command `payload` in a full VESC packet: a start byte
    (0x02 for short payloads up to 255 bytes, 0x03 + a 2-byte length for
    longer ones), the payload itself, a 2-byte CRC16 (from
    cubemars_crc16), and a trailing stop byte (0x03). Returns the
    complete bytes object ready to write to the serial port."""
    crc = cubemars_crc16(payload)
    crc_bytes = bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    if len(payload) <= 255:
        return bytes([2, len(payload)]) + payload + crc_bytes + bytes([3])
    length = len(payload)
    length_bytes = bytes([(length >> 8) & 0xFF, length & 0xFF])
    return bytes([3]) + length_bytes + payload + crc_bytes + bytes([3])


def cubemars_parse_response(raw: bytes):
    """Reverses cubemars_build_packet(): given raw bytes read back from
    the serial port, checks the start byte to figure out the header size
    and payload length, extracts the payload, and verifies the trailing
    CRC and stop byte. Returns the payload bytes if the frame is well
    formed and the CRC matches, otherwise None (covers truncated reads,
    corrupted bytes, or a malformed start byte)."""
    if len(raw) < 5:
        return None
    start = raw[0]
    if start == 2:
        length = raw[1]
        header_size = 2
    elif start == 3:
        length = (raw[1] << 8) | raw[2]
        header_size = 3
    else:
        return None
    payload = raw[header_size:header_size + length]
    crc_received = (raw[header_size + length] << 8) | raw[header_size + length + 1]
    stop = raw[header_size + length + 2]
    if stop != 3:
        return None
    if cubemars_crc16(payload) != crc_received:
        return None
    return payload


def cubemars_read_appconf():
    """Opens the port, reads app config, closes the port.
    Returns (payload_or_None, error_message_or_None) - same pattern as read_all_data."""
    port = cubemars_find_port()
    if port is None:
        return None, "CH340/R-Link not found. Plug it in."

    ser = serial.Serial(port, CUBEMARS_BAUD, timeout=1)
    result = None
    for attempt in range(3):
        ser.write(cubemars_build_packet(bytes([17])))  # COMM_GET_APPCONF
        time.sleep(0.3)
        response = ser.read(ser.in_waiting)
        result = cubemars_parse_response(response)
        if result is not None and len(result) >= 18:
            break
        ser.reset_input_buffer()
        time.sleep(0.3)
    ser.close()

    if result is None or len(result) < 18:
        return None, "Failed to read CubeMars app config."
    return result, None


def cubemars_write_appconf(new_controller_id, new_can_baud):
    """Reads current config, edits only the given fields, writes it back,
    then re-reads to verify. Returns (verify_payload_or_None, error_message_or_None)."""
    current, error = cubemars_read_appconf()
    if error:
        return None, error

    data = bytearray(current)
    data[0] = 16  # COMM_SET_APPCONF
    if new_controller_id is not None:
        data[5] = new_controller_id
    if new_can_baud is not None:
        data[17] = new_can_baud

    port = cubemars_find_port()
    if port is None:
        return None, "CH340/R-Link not found during write."

    ser = serial.Serial(port, CUBEMARS_BAUD, timeout=1)
    ser.write(cubemars_build_packet(bytes(data)))
    time.sleep(0.5)
    ser.close()

    verify, error = cubemars_read_appconf()
    return verify, error


def cubemars_read_mcconf():
    """Requests the full motor config (MCCONF) block from the controller
    via COMM_GET_MCCONF (command byte 14), retrying up to 3 times since
    the first reply after a cold connection sometimes gets dropped.
    Returns (payload_bytes, None) on success - the raw bytes, to be
    decoded field-by-field with cubemars_get_mcconf_field() - or
    (None, error_message) on failure."""
    port = cubemars_find_port()
    if port is None:
        return None, "CH340/R-Link not found. Plug it in."
    
    ser = serial.Serial(port, CUBEMARS_BAUD, timeout=1)
    result = None
    for attempt in range(3):
        ser.write(cubemars_build_packet(bytes([14])))
        time.sleep(0.3)
        response = ser.read(ser.in_waiting)
        result = cubemars_parse_response(response)
        if result is not None and len(result) >= 357:
            break
        ser.reset_input_buffer()
        time.sleep(0.3)
    ser.close()

    if result is None or len(result) < 357:
        return None, "Failed to read CubeMars motor config."
    return result, None


def cubemars_get_mcconf_field(mcconf, field_name):
    """Pulls one named field (e.g. 'speed_kp') out of a raw MCCONF byte
    blob returned by cubemars_read_mcconf(). Looks up the field's byte
    offset in CUBEMARS_MCCONF_OFFSETS, then unpacks 4 bytes there as a
    big-endian 32-bit float (struct format '>f'). Returns the float
    value."""
    offset = CUBEMARS_MCCONF_OFFSETS[field_name]
    return struct.unpack(">f", mcconf[offset:offset + 4])[0]


def cubemars_write_mcconf(changes: dict):
    """Read-modify-write for the motor config: reads the current MCCONF
    block, overwrites only the fields named in `changes` (a dict of
    field_name -> new float value, packed back to big-endian bytes at
    the right offset), sends the whole block back via COMM_SET_MCCONF
    (command byte 13), then re-reads to confirm. Returns whatever
    cubemars_read_mcconf() returns for that final confirmation read:
    (payload, None) or (None, error_message)."""
    current, error = cubemars_read_mcconf()
    if error:
        return None, error
    
    data=bytearray(current)
    data[0] = 13 # COMM_SET_MCCONF
    for field_name, new_value in changes.items():
        offset = CUBEMARS_MCCONF_OFFSETS[field_name]
        data[offset:offset + 4] = struct.pack(">f", new_value)
    
    port = cubemars_find_port()
    if port is None:
        return None, "CH340/R-Link not found during write."
    
    ser = serial.Serial(port, CUBEMARS_BAUD, timeout=1)
    ser.write(cubemars_build_packet((bytes(data))))
    time.sleep(0.5)
    ser.close()

    return cubemars_read_mcconf()


def build_cubemars_tab(parent):
    """Builds every widget on the CubeMars tab (labels, entry fields for
    controller ID / CAN baud / MCCONF values, Read All / Write All
    buttons) and wires them up to the cubemars_* protocol functions.
    `parent` is the tkinter Frame for this tab. Called once from main();
    doesn't return anything, just populates `parent` in place."""
    inner = tk.Frame(parent, width=640, height=830)
    inner.pack_propagate(False)
    inner.pack(anchor="nw", padx=10, pady=10)

    status_var = tk.StringVar(value="Not connected.")
    id_var = tk.StringVar(value="Controller ID: -")
    baud_var = tk.StringVar(value="CAN Baud: -")

    cubemars_state = {"connected": False}

    def poll_cubemars():
        """Runs every second (via parent.after) to check whether the
        CubeMars adapter is plugged in, and updates the status label
        when that connected/disconnected state changes. Just a USB
        presence check - no serial communication with the servo itself."""
        port = cubemars_find_port()
        if port and not cubemars_state["connected"]:
            cubemars_state["connected"] = True
            status_var.set(f"Servo detected on {port}. Click Read All.")
        elif not port and cubemars_state["connected"]:
            cubemars_state["connected"] = False
            status_var.set("Not connected.")
        parent.after(1000, poll_cubemars)

    poll_cubemars()

    tk.Label(inner, text="CubeMars AK80-8", font=("Segoe UI", 16, "bold")).pack(pady=(14, 6))
    tk.Label(inner, textvariable=id_var, font=("Segoe UI", 14)).pack(pady=1)
    tk.Label(inner, textvariable=baud_var, font=("Segoe UI", 14)).pack(pady=1)

    tk.Label(inner, text="New Controller ID (0-255):", font=("Segoe UI", 12)).pack(pady=(10, 0))
    new_id_entry = tk.Entry(inner, width=10, font=("Segoe UI", 12))
    new_id_entry.pack(pady=(2, 10))

    tk.Label(inner, text="New CAN Baud Rate:", font=("Segoe UI", 12)).pack()
    new_baud_var = tk.StringVar(value="-")
    baud_box = ttk.Combobox(inner, textvariable=new_baud_var, state="readonly",
                            values=["-"] + list(CUBEMARS_CAN_BAUD_LABELS.values()), width=10, font=("Segoe UI", 12))
    baud_box.pack(pady=(2, 14))

    tk.Label(inner, text="Motor Config", font=("Segoe UI", 14, "bold")).pack(pady=(10, 6))

    limits_frame = tk.Frame(inner)
    limits_frame.pack(pady=(0, 14))

    mcconf_entries = {}

    limit_pairs = [
        ("motor_max", "motor_min"),
        ("batt_max", "batt_min"),
        ("max_erpm", "min_erpm"),
    ]

    for row, (left_field, right_field) in enumerate(limit_pairs):
        tk.Label(limits_frame, text=CUBEMARS_DISPLAY_NAMES[left_field], font=("Segoe UI", 12)).grid(
            row=row, column=0, sticky="w", padx=(0, 4), pady=4)
        left_entry = tk.Entry(limits_frame, width=15, font=("Segoe UI", 12))
        left_entry.grid(row=row, column=1, padx=(0, 20), pady=2)
        mcconf_entries[left_field] = left_entry

        tk.Label(limits_frame, text=CUBEMARS_DISPLAY_NAMES[right_field], font=("Segoe UI", 12)).grid(
            row=row, column=2, sticky="w", padx=(0, 4), pady=4)
        right_entry = tk.Entry(limits_frame, width=15, font=("Segoe UI", 12))
        right_entry.grid(row=row, column=3, pady=2)
        mcconf_entries[right_field] = right_entry

    tk.Label(inner, text="Speed", font=("Segoe UI", 13, "bold")).pack(pady=(4, 4))
    speed_frame = tk.Frame(inner)
    speed_frame.pack(pady=(0, 10))

    for col, field in enumerate(["speed_kp", "speed_ki"]):
        short_label = CUBEMARS_DISPLAY_NAMES[field].replace("Speed ", "")
        tk.Label(speed_frame, text=short_label, font=("Segoe UI", 12)).grid(
            row=0, column=col * 2, sticky="w", padx=(0 if col == 0 else 20, 4))
        entry = tk.Entry(speed_frame, width=15, font=("Segoe UI", 12))
        entry.grid(row=0, column=col * 2 + 1)
        mcconf_entries[field] = entry

    tk.Label(inner, text="Position", font=("Segoe UI", 13, "bold")).pack(pady=(4, 4))
    position_frame = tk.Frame(inner)
    position_frame.pack(pady=(0, 14))

    for col, field in enumerate(["position_kp", "position_ki", "position_kd"]):
        short_label = CUBEMARS_DISPLAY_NAMES[field].replace("Position ", "")
        tk.Label(position_frame, text=short_label, font=("Segoe UI", 12)).grid(
            row=0, column=col * 2, sticky="w", padx=(0 if col == 0 else 20, 4))
        entry = tk.Entry(position_frame, width=15, font=("Segoe UI", 12))
        entry.grid(row=0, column=col * 2 + 1)
        mcconf_entries[field] = entry

    def on_read_all():
        """Handler for the 'Read All' button: reads app config (for
        controller ID and CAN baud) and motor config (for the limit/PID
        entry fields), and pushes every value into its widget. Bails out
        and shows the error in status_var if either read fails."""
        status_var.set("Reading...")
        parent.update()

        app_result, app_error = cubemars_read_appconf()
        if app_error:
            status_var.set(app_error)
            return
        id_var.set(f"Controller ID: {app_result[5]}")
        baud_var.set(f"CAN Baud: {CUBEMARS_CAN_BAUD_LABELS.get(app_result[17], 'unknown')}")

        mc_result, mc_error = cubemars_read_mcconf()
        if mc_error:
            status_var.set(mc_error)
            return
        for field_name, entry in mcconf_entries.items():
            set_entry(entry, cubemars_get_mcconf_field(mc_result, field_name))

        status_var.set("Read complete.")

    def on_write_all():
        """Handler for the 'Write All' button: validates every non-empty
        field (controller ID 0-255, a chosen baud label, and any MCCONF
        numbers typed in), then only sends the writes that are actually
        needed - app config and/or motor config - skipping calls
        entirely if nothing changed in that group. Refreshes the display
        with the servo's confirmed values after each write."""
        id_text = new_id_entry.get().strip()
        new_id = None
        if id_text:
            try:
                candidate = int(id_text)
                if 0 <= candidate <= 255:
                    new_id = candidate
                else:
                    status_var.set(f"{candidate} is out of range (0-255). ID not changed.")
                    return
            except ValueError:
                status_var.set(f"'{id_text}' is not a number. ID not changed.")
                return

        new_baud = None
        chosen_label = new_baud_var.get()
        if chosen_label != "-":
            for value, label in CUBEMARS_CAN_BAUD_LABELS.items():
                if label == chosen_label:
                    new_baud = value
                    break

        changes = {}
        for field_name, entry in mcconf_entries.items():
            text = entry.get().strip()
            if text:
                try:
                    changes[field_name] = float(text)
                except ValueError:
                    status_var.set(f"'{text}' is not a valid number for {field_name}.")
                    return
        
        if new_id is None and new_baud is None and not changes:
            status_var.set("No values entered - nothing to write.")
            return

        status_var.set("Writing...")
        parent.update()

        wrote_something = False

        if new_id is not None or new_baud is not None:
            app_result, app_error = cubemars_write_appconf(new_id, new_baud)
            if app_error:
                status_var.set(app_error)
                return
            id_var.set(f"Controller ID: {app_result[5]}")
            baud_var.set(f"CAN Baud: {CUBEMARS_CAN_BAUD_LABELS.get(app_result[17], 'unknown')}")
            wrote_something = True

        if changes:
            mc_result, mc_error = cubemars_write_mcconf(changes)
            if mc_error:
                status_var.set(mc_error)
                return
            for field_name, entry in mcconf_entries.items():
                            set_entry(entry, cubemars_get_mcconf_field(mc_result, field_name))
            wrote_something = True

        if wrote_something:
            status_var.set("Written and verified.")

    tk.Button(inner, text="Read All", command=on_read_all,
             font=("Segoe UI", 12), width=18).pack(pady=(6, 4))
    tk.Button(inner, text="Write All", command=on_write_all,
             font=("Segoe UI", 12), width=18).pack(pady=(0, 14))

    tk.Label(inner, textvariable=status_var, fg="gray", font=("Segoe UI", 14),
             wraplength=500, justify="center").pack(pady=(10, 8))
    

# ---------------------------------------------------------------------
# Window
# ---------------------------------------------------------------------

def main():
    """Entry point. Builds the main window and the two-tab Notebook,
    builds every widget and handler for the Hitec tab directly inline
    (IDs, profile selector, vehicle diagram, settings fields, save
    buttons), delegates the CubeMars tab to build_cubemars_tab(), starts
    the two background polling loops (poll(), poll_global_status()),
    and finally hands control to tkinter's mainloop(). Doesn't return
    until the window is closed."""
    root = tk.Tk() # Creates a new window
    root.title("Servo Monitor") # Title of the GUI
    root.geometry("1250x820") # Size of the GUI window

    root.option_add("*TCombobox*Listbox.font", ("Segoe UI", 11)) # Font size for the text in the dropdown menu of the profile selector

    global_status_var = tk.StringVar(value="No servo connected.")
    tk.Label(root, textvariable=global_status_var, font=("Segoe UI", 11, "bold"),
             fg="#555555").pack(side=tk.TOP, fill=tk.X, pady=(4, 0))

    notebook = ttk.Notebook(root)
    notebook.pack(fill=tk.BOTH, expand = True)

    hitec_tab = tk.Frame(notebook)
    cubemars_tab = tk.Frame(notebook)
    notebook.add(hitec_tab, text="Hitec")
    notebook.add(cubemars_tab, text="CubeMars")

    style = ttk.Style()
    style.configure("TNotebook.Tab", font=("Segoe UI", 13), padding=(20,8))

    def on_tab_change(event):
        selected = event.widget.select()
        tab_text = event.widget.tab(selected, "text")
        if tab_text == "CubeMars":
            root.geometry("620x840")
        else:
            root.geometry("1250x860")
    
    notebook.bind("<<NotebookTabChanged>>", on_tab_change)    
        

    # Creating two columns in the window, content_frame for the parameters and diagram_panel for the airships visual
    content_frame = tk.Frame(hitec_tab)
    content_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=10, pady=10)
    diagram_panel = tk.Frame(hitec_tab)
    diagram_panel.pack(side=tk.RIGHT, fill=tk.BOTH, padx=10, pady=10)


    status_var = tk.StringVar(value="Waiting for servo...") # The default status when code run

    # Creates a label for ID1 and ID2
    id1_var = tk.StringVar(value="ID1 (Actuator ID): -")
    id2_var = tk.StringVar(value="ID2 (CAN Node ID): -")

    # Creates a label for Servo IDs and the actual ID's when button is pressed
    tk.Label(content_frame, text="Servo IDs", font=("Segoe UI", 16, "bold")).pack(pady=(14, 6))
    tk.Label(content_frame, textvariable=id1_var, font=("Segoe UI", 14)).pack(pady=1)
    tk.Label(content_frame, textvariable=id2_var, font=("Segoe UI", 14)).pack(pady=1)

    def on_read_id(): # This function is giving information as soon as the Read ID button is pressed 
        """Handler for the 'Read ID' button: calls read_ids_only(),
        updates the ID1/ID2 labels with whatever came back (or 'read
        failed' per-ID if a value was None), and updates the status
        line."""
        status_var.set("Reading ID...")
        root.update()
        ids, error = read_ids_only()
        if error:
            status_var.set(error)
            return
        id1, id2 = ids
        id1_var.set(f"ID1 (Actuator ID): {id1 if id1 is not None else 'read failed'}")
        id2_var.set(f"ID2 (CAN Node ID): {id2 if id2 is not None else 'read failed'}")
        status_var.set("ID read.")

    tk.Button(content_frame, text="Read ID", command=on_read_id,
             font=("Segoe UI", 12), width=12).pack(pady=(2, 4)) # This is the code to create the Read ID button 


    tk.Label(content_frame, text="Servo Profile", font=("Segoe UI", 16, "bold")).pack(pady=(14, 4)) # A label to indicate the Servo Profiles

    profile_frame = tk.Frame(content_frame) 
    profile_frame.pack()

    profiles = load_profiles() # Creating a variable called profiles and assigning it the profile dict
    profile_var = tk.StringVar(value="-")
    profile_box = ttk.Combobox(profile_frame, textvariable=profile_var,
                               values=list(profiles.keys()), width=44, state="readonly", font =("Segoe UI", 11))
    profile_box.grid(row=0, column=0, columnspan=3, padx=4, pady=(0, 6)) # The placement of the box 

    new_profile_entry = tk.Entry(profile_frame, width=24, font = ("Segoe UI", 11))
    new_profile_entry.grid(row=1, column=0, padx=4)

    tk.Button(profile_frame, text="Add", width=6, font=("Segoe UI", 11),
             command=lambda: on_add_profile()).grid(row=1, column=1, padx=4) # Button to add a profile 
    tk.Button(profile_frame, text="Delete", width=7, font=("Segoe UI", 11),
             command=lambda: on_delete_profile()).grid(row=1, column=2, padx=4) # Button to delete a profile

    tk.Button(content_frame, text="Save to profile only", command=lambda: on_save_to_profile(),
             font=("Segoe UI", 13), width=20).pack(pady=(6, 4)) # Button to save the setting to the servo only

    # --- Vehicle diagram (always visible, embedded in the main window) ---
    # Drawn directly with Canvas shapes (no external image file), so it
    # looks like a native part of the app rather than a pasted-in photo.
    # Because WE define this coordinate system, the inner/outer marker
    # positions below are calculated exactly rather than estimated off
    # a picture - change BODY_R / TIP_R / TICK_R to reshape the whole
    # diagram if the real vehicle's proportions differ.
    diagram_state = {"canvas": None, "marker": None}

    BACK_CENTER = (240, 250)   # back-view circle center
    BODY_R = 90                   # back-view body radius
    TIP_R = 200                   # arm length (tip distance from center)
    TICK_R = 138   # where the inner/outer tick mark sits
    ARM_WIDTH = 30

    SIDE_CENTER = (240, 600)      # side-view ellipse center
    SIDE_RX, SIDE_RY = 180, 80 # Size of the side view oval 

    # Angles (degrees, 0=right/east, increasing counter-clockwise) where
    # the vent and tether sit on the side-view ellipse's boundary - this
    # is what makes them look physically attached rather than floating.
    # Changing these numbers is how you move the vent/tether around the
    # ellipse - they're used to CALCULATE the coordinates below, which
    # then get written into POSITION_COORDS so the drawn shape and the
    # red marker dot always agree on where these two actually are.
    VENT_ANGLE_DEG = 35 # Position of the vent drawing
    TETHER_ANGLE_DEG = 8 # Position of the tether drawing 

    def _point_on_ellipse(center, rx, ry, angle_deg):
        """Converts a polar angle on an ellipse (center, radii rx/ry)
        into (x, y) canvas coordinates on that ellipse's boundary. Used
        to place the vent/tether markers exactly on the drawn hull line
        rather than guessing pixel coordinates by eye."""
        rad = math.radians(angle_deg)
        cx, cy = center
        return (cx + rx * math.cos(rad), cy + ry * math.sin(rad))

    POSITION_COORDS["emergency_vent"] = _point_on_ellipse(
        SIDE_CENTER, SIDE_RX, SIDE_RY, VENT_ANGLE_DEG)
    POSITION_COORDS["safety_tether"] = _point_on_ellipse(
        SIDE_CENTER, SIDE_RX, SIDE_RY, TETHER_ANGLE_DEG)

    ARM_DIRECTIONS = {
        # unit vectors pointing from the body center toward each arm
        "top_left": (-0.7071, -0.7071),
        "top_right": (0.7071, -0.7071),
        "bottom_left": (-0.7071, 0.7071),
        "bottom_right": (0.7071, 0.7071),
    }

    def _point_along_arm(direction, radius):
        """Given a unit vector `direction` (from ARM_DIRECTIONS) and a
        distance `radius` from the body center, returns the (x, y)
        canvas point that far out along that arm. Used to compute the
        inner/outer marker positions for each of the 4 arms."""
        dx, dy = direction
        cx, cy = BACK_CENTER

        return (cx + radius * dx, cy + radius * dy)
    
    for name, direction in ARM_DIRECTIONS.items():
        inner_radius = (BODY_R + TICK_R) / 2   # midpoint of the inner segment
        outer_radius = (TICK_R + TIP_R) / 2    # midpoint of the outer segment
        POSITION_COORDS[f"{name}_inner"] = _point_along_arm(direction, inner_radius)
        POSITION_COORDS[f"{name}_outer"] = _point_along_arm(direction, outer_radius)

    def update_diagram_marker():
        """Redraws the green dot on the vehicle diagram to match
        whichever profile is currently selected. Removes any existing
        marker first, then (if the selected profile's name maps to a
        known position via match_profile_to_position) draws a new one
        there. Called whenever the selected profile changes, or when a
        profile is deleted / the servo disconnects."""
        canvas = diagram_state.get("canvas")
        if canvas is None:
            return

        if diagram_state.get("marker") is not None:
            canvas.delete(diagram_state["marker"])
            diagram_state["marker"] = None

        name = profile_var.get()
        if name and name != "-":
            key = match_profile_to_position(name)
            if key:
                x, y = POSITION_COORDS[key]
                if key in ("safety_tether", "emergency_vent"): # Changes the size of the marker based on which profile is selected
                    r = 5.5
                else:
                    r = 8
                diagram_state["marker"] = canvas.create_oval(
                    x - r, y - r, x + r, y + r,
                    fill="green",outline ="green", width=1) # The fill of the marker

    def build_diagram_canvas(parent):
        """Draws the static vehicle diagram once: a back view (body
        circle + 4 rotated-rectangle arms with inner/outer tick marks)
        and a side view (hull ellipse + emergency vent grille + safety
        tether rectangle). Everything is drawn with Canvas primitives
        using the geometry constants defined above (BACK_CENTER, BODY_R,
        etc.) - there's no external image file. Stores the canvas in
        diagram_state so update_diagram_marker() can add/remove the
        selection dot on top of it later."""
        canvas = tk.Canvas(parent, width=480, height=780, bg="#f0f0f0",
                          highlightthickness=1, highlightbackground="#f0f0f0")
        canvas.pack(pady=(4, 10))
        diagram_state["canvas"] = canvas

        cx, cy = BACK_CENTER

        canvas.create_text(cx, 50, text="Back View",
                          font=("Segoe UI", 14, "italic"), fill="#000000")

        # --- back view: body + 4 arms (true rotated rectangles) with a
        #     tick mark on each showing the inner/outer split ---
        for dx, dy in ARM_DIRECTIONS.values():
            perp_dx, perp_dy = -dy, dx
            hw = ARM_WIDTH / 2
            p1x, p1y = cx + BODY_R * dx, cy + BODY_R * dy   # arm base (at body)
            p2x, p2y = cx + TIP_R * dx, cy + TIP_R * dy     # arm tip

            corners = [
                p1x + perp_dx * hw, p1y + perp_dy * hw,
                p2x + perp_dx * hw, p2y + perp_dy * hw,
                p2x - perp_dx * hw, p2y - perp_dy * hw,
                p1x - perp_dx * hw, p1y - perp_dy * hw,
            ]
            canvas.create_polygon(corners, outline="#333333", width=2, fill="#f2f2f2")

            # tick mark: short perpendicular line at the inner/outer split
            tx, ty = cx + TICK_R * dx, cy + TICK_R * dy
            half = hw
            canvas.create_line(tx - perp_dx * half, ty - perp_dy * half,
                               tx + perp_dx * half, ty + perp_dy * half,
                               width=2, fill="#333333")

        canvas.create_oval(cx - BODY_R, cy - BODY_R, cx + BODY_R, cy + BODY_R,
                          outline="#333333", width=3, fill="#f0f0f0")

        # --- side view: body ellipse + emergency vent + safety tether,
        #     both positioned ON the ellipse boundary so they read as
        #     physically attached rather than floating nearby ---
        ex, ey = SIDE_CENTER
        canvas.create_text(ex, ey - SIDE_RY - 45, text="Side View",
                          font=("Segoe UI", 14, "italic"), fill="#000000")
        canvas.create_oval(ex - SIDE_RX, ey - SIDE_RY, ex + SIDE_RX, ey + SIDE_RY,
                          outline="#333333", width=3)

        # inner arc (the extra curve visible inside the body in the
        # original sketch, like an underside/hull line). Adjust
        # INNER_ARC_* below if the curve's shape/position needs tuning.
        INNER_ARC_RX = SIDE_RX * 0.75
        INNER_ARC_RY = SIDE_RY * 0.7
        inner_cx = ex - SIDE_RX * 0.05
        inner_cy = ey - SIDE_RY * -0.9
        canvas.create_arc(inner_cx - INNER_ARC_RX, inner_cy - INNER_ARC_RY,
                         inner_cx + INNER_ARC_RX, inner_cy + INNER_ARC_RY,
                         start=15, extent=145, style=tk.ARC,
                         outline="#333333", width=2)

        # emergency vent: a small grille icon (3x3 grid) straddling the
        # ellipse boundary at VENT_ANGLE_DEG
        vx, vy = POSITION_COORDS["emergency_vent"]
        vs = 10  # half-size of the vent square
        canvas.create_rectangle(vx - vs, vy - vs, vx + vs, vy + vs,
                               outline="#333333", width=2, fill="white")
        for i in (1, 2):  # 3x3 grille lines
            gx = vx - vs + i * (2 * vs) / 3
            canvas.create_line(gx, vy - vs, gx, vy + vs, fill="#333333")
            gy = vy - vs + i * (2 * vs) / 3
            canvas.create_line(vx - vs, gy, vx + vs, gy, fill="#333333")

        # safety tether: a rectangle straddling the ellipse boundary at
        # TETHER_ANGLE_DEG, oriented along that same radial direction
        tx3, ty3 = POSITION_COORDS["safety_tether"]
        t_rad = math.radians(TETHER_ANGLE_DEG)
        t_dx, t_dy = math.cos(t_rad), math.sin(t_rad)   # outward direction
        t_perp_dx, t_perp_dy = -t_dy, t_dx
        t_len, t_wid = 10, 10 # Size of the safety tether rectangle (half-length, half-width)
        t_corners = [
            tx3 + t_dx * t_len + t_perp_dx * t_wid, ty3 + t_dy * t_len + t_perp_dy * t_wid,
            tx3 - t_dx * t_len + t_perp_dx * t_wid, ty3 - t_dy * t_len + t_perp_dy * t_wid,
            tx3 - t_dx * t_len - t_perp_dx * t_wid, ty3 - t_dy * t_len - t_perp_dy * t_wid,
            tx3 + t_dx * t_len - t_perp_dx * t_wid, ty3 + t_dy * t_len - t_perp_dy * t_wid,
        ]
        canvas.create_polygon(t_corners, outline="#333333", width=2, fill="#f0f0f0")

    build_diagram_canvas(diagram_panel)

    tk.Label(content_frame, text="Settings (raw values)", font=("Segoe UI", 14, "bold")).pack(pady=(14, 4))

    fields_frame = tk.Frame(content_frame)
    fields_frame.pack(padx=10)
    # Two side-by-side groups of rows (label+value), each its own pair
    # of grid columns - this is what lets the settings wrap into a
    # second column instead of stacking into one long list.
    WRAP_AFTER = 6
    LEFT_LABEL_WIDTH = 24    # column-pair 0 (left side)
    RIGHT_LABEL_WIDTH = 21   # column-pair 1 (right side) - smaller, so
                             # the box sits closer to the text there
    COLUMN_GAP = 24  # extra space BEFORE the right column starts - the
                     # number to change to push the two columns apart
    fields_frame.columnconfigure(1, minsize=130)
    fields_frame.columnconfigure(3, minsize=130)

    def grid_slot(index):
        """Turns a plain 0,1,2,3... counter into (row, label_col,
        value_col, label_width, label_padx) - after WRAP_AFTER rows,
        it starts a new column pair instead of continuing downward."""
        row = index % WRAP_AFTER
        pair = index // WRAP_AFTER
        label_width = LEFT_LABEL_WIDTH if pair == 0 else RIGHT_LABEL_WIDTH
        label_padx = (4, 4) if pair == 0 else (COLUMN_GAP, 4)
        return row, pair * 2, pair * 2 + 1, label_width, label_padx

    # (key, label, register) - plain number entry fields
    FIELD_INFO = [
        ("pos_mid", "Pos Mid (-16384..16384)", 0xC2),
        ("pos_max", "Pos Max (-16384..16384)", 0xB0),
        ("pos_min", "Pos Min (-16384..16384)", 0xB2),
        ("speed", "Speed (0-4095)", 0x54),
        ("speed_up", "Time Spd Up (ms)", 0xDC),
        ("speed_dn", "Time Spd Dn (ms)", 0xDE),
        ("stream_time", "Stream Time (0-12000)", 0x2E),
    ]

    SAMPLE_POINT_REGISTER = 0x40
    SAMPLE_POINT_LABELS = {0: "50", 1: "87.5"}
    SAMPLE_POINT_VALUES = {label: raw for raw, label in SAMPLE_POINT_LABELS.items()}

    BAUD_RATE_REGISTER = 0x38
    BAUD_RATE_LABELS = {
        0: "1000", 1: "800", 2: "750", 3: "500",
        4: "400", 5: "250", 6: "200", 7: "150", 8: "125",
    }
    BAUD_RATE_VALUES = {label: raw for raw, label in BAUD_RATE_LABELS.items()}

    STREAM_MODE_REGISTER = 0x30
    STREAM_MODE_LABELS = {0: "Off", 1: "On"}
    STREAM_MODE_VALUES = {label: raw for raw, label in STREAM_MODE_LABELS.items()}

    # IDs: written with everything else, but only take effect after a
    # power cycle (per the manual), so their true confirmation is the
    # display at the top after replug. DroneCAN ranges.
    ID1_REGISTER, ID2_REGISTER = 0x32, 0x3E

    entries = {}
    next_index = 0  # shared counter across ALL rows below

    # --- ID rows (entry fields, validated 1-127 / 0-127 on save) ---
    # Moved to run FIRST so ID1/ID2 are the first two rows shown.
    id_entries = {}
    ID_WRITE_INFO = [
        ("ID1 (Actuator ID)", ID1_REGISTER, 1, 127),
        ("ID2 (CAN Node ID)", ID2_REGISTER, 0, 127),
    ]
    for label, register, lo_limit, hi_limit in ID_WRITE_INFO:
        r, lc, vc, lw, lpad = grid_slot(next_index); next_index += 1
        tk.Label(fields_frame, text=f"{label} ({lo_limit}-{hi_limit})",
                 width=lw, font=("Segoe UI", 11), anchor="w").grid(row=r, column=lc, sticky="w", padx=lpad, pady=3)
        entry = tk.Entry(fields_frame, width=11, font=("Segoe UI", 11))
        entry.insert(0, "-")
        entry.grid(row=r, column=vc, padx=4, pady=3, sticky="ew")
        id_entries[label] = entry

    # --- plain entry rows ---
    for key, label, register in FIELD_INFO:
        row, label_col, value_col, label_width, label_padx = grid_slot(next_index)
        next_index += 1
        tk.Label(fields_frame, text=label, width=label_width, font=("Segoe UI", 11), anchor="w").grid(
            row=row, column=label_col, sticky="w", padx=label_padx, pady=3)
        entry = tk.Entry(fields_frame, width=11, font=("Segoe UI", 11))
        entry.insert(0, "-")
        entry.grid(row=row, column=value_col, padx=4, pady=3, sticky="ew")
        entries[key] = (entry, register)

    # --- dropdown rows ---

    sm_row, sm_lc, sm_vc, sm_lw, sm_lpad = grid_slot(next_index); next_index += 1
    tk.Label(fields_frame, text="Stream Mode", width=sm_lw, font=("Segoe UI", 11), anchor="w").grid(
        row=sm_row, column=sm_lc, sticky="w", padx=sm_lpad, pady=3)
    stream_mode_var = tk.StringVar(value="-")
    stream_mode_menu = tk.OptionMenu(fields_frame, stream_mode_var, *STREAM_MODE_LABELS.values())
    stream_mode_menu.config(width=7, font=("Segoe UI", 11))
    stream_mode_menu.grid(row=sm_row, column=sm_vc, padx=4, pady=3, sticky="ew")
    stream_mode_menu["menu"].config(font=("Segoe UI", 11))

    sp_row, sp_lc, sp_vc, sp_lw, sp_lpad = grid_slot(next_index); next_index += 1
    tk.Label(fields_frame, text="Sample Point [%]", width=sp_lw, font=("Segoe UI", 11), anchor="w").grid(
        row=sp_row, column=sp_lc, sticky="w", padx=sp_lpad, pady=3)
    sample_point_var = tk.StringVar(value="-")
    sample_point_menu = tk.OptionMenu(fields_frame, sample_point_var, *SAMPLE_POINT_LABELS.values())
    sample_point_menu.config(width=7, font=("Segoe UI", 11))
    sample_point_menu.grid(row=sp_row, column=sp_vc, padx=4, pady=3, sticky="ew")
    sample_point_menu["menu"].config(font=("Segoe UI", 11))

    br_row, br_lc, br_vc, br_lw, br_lpad = grid_slot(next_index); next_index += 1
    tk.Label(fields_frame, text="Baud Rate [kbps]", width=br_lw, font=("Segoe UI", 11), anchor="w").grid(
        row=br_row, column=br_lc, sticky="w", padx=br_lpad, pady=3)
    baud_rate_var = tk.StringVar(value="-")
    baud_rate_menu = tk.OptionMenu(fields_frame, baud_rate_var, *BAUD_RATE_LABELS.values())
    baud_rate_menu.config(width=7, font=("Segoe UI", 11))
    baud_rate_menu.grid(row=br_row, column=br_vc, padx=4, pady=3, sticky="ew")
    baud_rate_menu["menu"].config(font=("Segoe UI",11))

    # --- the single Save button ---

    def gather_and_validate():
        """Reads every field, validates it, and returns two views of the
        same data: `to_write` (register -> value, for writing to the
        servo) and `settings` (profile-key -> value, for saving to the
        JSON profile). Returns (to_write, settings, error_message).
        If error_message is not None, nothing was written anywhere -
        both callers must check this before doing anything."""
        to_write = {}   # register -> value
        settings = {}   # profile key -> value

        for key, (entry, register) in entries.items():
            raw = entry.get().strip()
            try:
                value = int(raw)
            except ValueError:
                return None, None, f"'{raw}' for {key} isn't a whole number. Nothing saved."
            to_write[register] = value
            settings[key] = value

        sp_label = sample_point_var.get()
        if sp_label not in SAMPLE_POINT_VALUES:
            return None, None, "Pick a Sample Point first. Nothing saved."
        to_write[SAMPLE_POINT_REGISTER] = SAMPLE_POINT_VALUES[sp_label]
        settings["sample_point"] = SAMPLE_POINT_VALUES[sp_label]

        br_label = baud_rate_var.get()
        if br_label not in BAUD_RATE_VALUES:
            return None, None, "Pick a Baud Rate first. Nothing saved."
        to_write[BAUD_RATE_REGISTER] = BAUD_RATE_VALUES[br_label]
        settings["baud_rate"] = BAUD_RATE_VALUES[br_label]

        sm_label = stream_mode_var.get()
        if sm_label not in STREAM_MODE_VALUES:
            return None, None, "Pick a Stream Mode first. Nothing saved."
        to_write[STREAM_MODE_REGISTER] = STREAM_MODE_VALUES[sm_label]
        settings["stream_mode"] = STREAM_MODE_VALUES[sm_label]

        for label, register, lo_limit, hi_limit in ID_WRITE_INFO:
            raw = id_entries[label].get().strip()
            try:
                value = int(raw)
            except ValueError:
                return None, None, f"'{raw}' for {label} isn't a whole number. Nothing saved."
            if not (lo_limit <= value <= hi_limit):
                return None, None, f"{label} must be {lo_limit}-{hi_limit}. Nothing saved."
            to_write[register] = value
            settings["id1" if label.startswith("ID1") else "id2"] = value

        return to_write, settings, None

    def on_save_to_servo():
        """Writes the current field values to the real servo and
        confirms them. Does NOT touch the saved profile file."""
        to_write, _settings, error = gather_and_validate()
        if error:
            status_var.set(error)
            return

        status_var.set(f"Writing {len(to_write)} settings to servo...")
        root.update()

        confirmed, error = write_all(to_write)
        if error:
            status_var.set(error)
            return

        # Update the UI from what the servo actually reports back.
        failed = []
        for key, (entry, register) in entries.items():
            back = confirmed.get(register)
            set_entry(entry, back, placeholder="??")
            if back is None:
                failed.append(key)

        sp_back = confirmed.get(SAMPLE_POINT_REGISTER)
        sample_point_var.set(SAMPLE_POINT_LABELS.get(sp_back, "??" if sp_back is None else str(sp_back)))
        if sp_back is None:
            failed.append("sample_point")

        br_back = confirmed.get(BAUD_RATE_REGISTER)
        baud_rate_var.set(BAUD_RATE_LABELS.get(br_back, "??" if br_back is None else str(br_back)))
        if br_back is None:
            failed.append("baud_rate")

        sm_back = confirmed.get(STREAM_MODE_REGISTER)
        stream_mode_var.set(STREAM_MODE_LABELS.get(sm_back, "??" if sm_back is None else str(sm_back)))
        if sm_back is None:
            failed.append("stream_mode")

        if failed:
            status_var.set(f"Written to servo, but couldn't confirm: {', '.join(failed)}. "
                           "IDs need an unplug/replug to apply.")
        else:
            status_var.set("Written to servo and confirmed. "
                           "Press Read ID button to see updated ID's")

    def on_save_to_profile():
        """Saves the current field values into the selected profile's
        JSON entry. Does NOT touch the servo at all - no serial
        communication happens here."""
        profile_name = profile_var.get()
        if not profile_name or profile_name == "-" or profile_name not in profiles:
            status_var.set("Select or add a profile first. Nothing saved.")
            return

        _to_write, settings, error = gather_and_validate()
        if error:
            status_var.set(error)
            return

        profiles[profile_name] = settings
        save_profiles(profiles)
        status_var.set(f"Saved to profile '{profile_name}'.")

    tk.Button(content_frame, text="Save to servo only", command=on_save_to_servo,
             font=("Segoe UI", 13), width=20).pack(pady=(10, 4))

    tk.Label(content_frame, textvariable=status_var, fg="gray", font=("Segoe UI", 15),
             wraplength=380, justify="center").pack(pady=(8, 8))

    state = {"connected_port": None}

    def fill_settings_fields(settings):
        """Populate the settings fields from a plain dict of
        key -> raw value - used for loading a SAVED profile. Does not
        touch the ID display at the top (that's live-read separately)."""
        for key, (entry, _register) in entries.items():
            val = settings.get(key)
            set_entry(entry, val)

        sp_val = settings.get("sample_point")
        sample_point_var.set(SAMPLE_POINT_LABELS.get(sp_val, "-" if sp_val is None else str(sp_val)))

        br_val = settings.get("baud_rate")
        baud_rate_var.set(BAUD_RATE_LABELS.get(br_val, "-" if br_val is None else str(br_val)))

        sm_val = settings.get("stream_mode")
        stream_mode_var.set(STREAM_MODE_LABELS.get(sm_val, "-" if sm_val is None else str(sm_val)))

        for label, source_key in (("ID1 (Actuator ID)", "id1"), ("ID2 (CAN Node ID)", "id2")):
            entry = id_entries[label]
            val = settings.get(source_key)
            set_entry(entry, val)

    def on_profile_selected(_event=None):
        """Bound to the profile dropdown's <<ComboboxSelected>> event.
        Selecting a profile loads its SAVED settings into the fields.
        No serial communication happens here - the servo isn't touched
        until you click Save."""
        name = profile_var.get()
        if name and name != "-" and name in profiles:
            fill_settings_fields(profiles[name])
            status_var.set(f"'{name}' profile loaded. Edit as needed, then Save "
                           "to write it to the servo.")
        update_diagram_marker()

    def on_add_profile():
        """Handler for the 'Add' button: creates a new profile with
        DEFAULT_PROFILE_SETTINGS if the typed name doesn't already
        exist, saves profiles.json, refreshes the dropdown's list of
        values, selects the new profile, and loads its fields (via
        on_profile_selected). If the name already exists, just selects
        and loads the existing one instead of overwriting it."""
        name = new_profile_entry.get().strip()
        if not name:
            status_var.set("Type a profile name first.")
            return
        if name not in profiles:
            profiles[name] = dict(DEFAULT_PROFILE_SETTINGS)
            save_profiles(profiles)
            profile_box.config(values=list(profiles.keys()))
        profile_var.set(name)
        new_profile_entry.delete(0, tk.END)
        on_profile_selected()

    def on_delete_profile():
        """Handler for the 'Delete' button.
        Deletes the currently selected profile from the saved file.
        Does not touch the servo."""
        name = profile_var.get()
        if not name or name == "-" or name not in profiles:
            status_var.set("Select a profile to delete first.")
            return

        del profiles[name]
        save_profiles(profiles)
        profile_box.config(values=list(profiles.keys()))
        profile_var.set("-")

        for entry, _register in entries.values():
            set_entry(entry, None)
        sample_point_var.set("-")
        baud_rate_var.set("-")
        stream_mode_var.set("-")
        for entry in id_entries.values():
            set_entry(entry, None)

        update_diagram_marker()
        status_var.set(f"Deleted profile '{name}'.")

    profile_box.bind("<<ComboboxSelected>>", on_profile_selected)

    def poll():
        """Runs every second (via root.after) to detect the Hitec servo
        being plugged in or unplugged, purely by USB presence - it never
        opens the serial port or talks to the servo. Clears all fields
        and shows 'Waiting for servo...' on disconnect; on a fresh
        connection, resets the profile selector to '-' and prompts the
        user to pick which profile this servo is, since the app can't
        tell which physical actuator just appeared."""
        port = find_hitec_port()

        if port is None:
            if state["connected_port"] is not None:
                state["connected_port"] = None
                for entry, _r in entries.values():
                    set_entry(entry, None)
                sample_point_var.set("-")
                baud_rate_var.set("-")
                stream_mode_var.set("-")
                for entry in id_entries.values():
                    set_entry(entry, None)
                profile_var.set("-")
                update_diagram_marker()
                status_var.set("Waiting for servo...")

        elif port != state["connected_port"]:
            # A new servo just appeared. This is just a USB presence
            # check (no serial communication at all) - select which
            # profile this is, and the fields load from the saved
            # profile, not a live read.
            state["connected_port"] = port
            profile_var.set("-")
            update_diagram_marker()
            status_var.set(f"Servo detected on {port}. Select which profile this is.")

        root.after(1000, poll)

    def poll_global_status():
        """Runs every second (via root.after) to update the top-of-window
        status bar with which servo(s) are currently connected - Hitec,
        CubeMars, both, or neither - based on a simple USB presence
        check for each adapter."""
        hitec_port = find_hitec_port()
        cubemars_port = cubemars_find_port()

        if hitec_port and cubemars_port:
            global_status_var.set(f"Hitec on {hitec_port}, CubeMars on {cubemars_port} — both connected.")
        elif hitec_port:
            global_status_var.set(f"Hitec servo connected on {hitec_port}.")
        elif cubemars_port:
            global_status_var.set(f"CubeMars servo connected on {cubemars_port}.")
        else:
            global_status_var.set("No servo connected.")

        root.after(1000, poll_global_status)

    build_cubemars_tab(cubemars_tab)
    poll_global_status()
    poll()
    root.mainloop()

if __name__ == "__main__":
    main()