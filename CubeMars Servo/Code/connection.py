import serial
from serial.tools import list_ports
import time
import struct

TARGET_VID = 0x1A86
TARGET_PID = 0x7523
baud_rate = 921600

CAN_BAUD_LABELS = {
    0: "125K", 1: "250K", 2: "500K", 3: "1M",
}

MCCONF_OFFSETS = {
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


def find_port():
    for port in list_ports.comports():
        if port.pid == TARGET_PID and port.vid == TARGET_VID:
            return port.device
    return None


def crc16(data: bytes) -> int:
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


def build_packet(payload: bytes) -> bytes:
    crc = crc16(payload)
    crc_bytes = bytes([(crc >> 8) & 0xFF, crc & 0xFF])

    if len(payload) <= 255:
        return bytes([2, len(payload)]) + payload + crc_bytes + bytes([3])
    else:
        length = len(payload)
        length_bytes = bytes([(length >> 8) & 0xFF, length & 0xFF])
        return bytes([3]) + length_bytes + payload + crc_bytes + bytes([3])


def parse_response(raw: bytes):
    if len(raw) < 5:
        print("Too short to be a valid packet")
        return None

    start = raw[0]

    if start == 2:
        length = raw[1]
        header_size = 2
    elif start == 3:
        length = (raw[1] << 8) | raw[2]
        header_size = 3
    else:
        print("Bad start byte")
        return None

    payload = raw[header_size:header_size + length]
    crc_received = (raw[header_size + length] << 8) | raw[header_size + length + 1]
    stop = raw[header_size + length + 2]

    if stop != 3:
        print("Bad stop byte")
        return None

    crc_calculated = crc16(payload)
    if crc_calculated != crc_received:
        print(f"CRC mismatch: got {crc_received}, expected {crc_calculated}")
        return None

    return payload


# --- App config (Controller ID, CAN Baud Rate) ---

def read_appconf(ser, wait=0.3, retries=3):
    for attempt in range(1, retries + 1):
        packet = build_packet(bytes([17]))  # COMM_GET_APPCONF
        ser.write(packet)

        time.sleep(wait)
        response = ser.read(ser.in_waiting)
        result = parse_response(response)

        if result is not None and len(result) >= 18:
            return result

        print(f"Attempt {attempt} failed (got {len(response)} bytes), retrying...")
        ser.reset_input_buffer()
        time.sleep(0.3)

    print("All retry attempts failed.")
    return None


def write_appconf(ser, current_appconf: bytes, new_controller_id=None, new_can_baud=None):
    data = bytearray(current_appconf)
    data[0] = 16  # COMM_SET_APPCONF

    if new_controller_id is not None:
        data[5] = new_controller_id
    if new_can_baud is not None:
        data[17] = new_can_baud

    packet = build_packet(bytes(data))
    ser.write(packet)
    print("Sent SET_APPCONF:", f"{len(packet)} bytes")

    time.sleep(0.5)
    ser.reset_input_buffer()
    return read_appconf(ser, wait=0.5)


# --- Motor config (current limits, speed/position PID) ---

def read_mcconf(ser, wait=0.3, retries=3):
    for attempt in range(1, retries + 1):
        packet = build_packet(bytes([14]))  # COMM_GET_MCCONF
        ser.write(packet)

        time.sleep(wait)
        response = ser.read(ser.in_waiting)
        result = parse_response(response)

        if result is not None and len(result) >= 357:
            return result

        print(f"Attempt {attempt} failed (got {len(response)} bytes), retrying...")
        ser.reset_input_buffer()
        time.sleep(0.3)

    print("All retry attempts failed.")
    return None


def get_mcconf_field(mcconf: bytes, field_name: str) -> float:
    offset = MCCONF_OFFSETS[field_name]
    return struct.unpack(">f", mcconf[offset:offset + 4])[0]


def write_mcconf(ser, current_mcconf: bytes, changes: dict, wait=0.5):
    """changes: dict like {"speed_kp": 0.005, "position_kd": 0.0005}
    Only the fields you include get modified - everything else is sent back untouched."""
    data = bytearray(current_mcconf)
    data[0] = 13  # COMM_SET_MCCONF

    for field_name, new_value in changes.items():
        offset = MCCONF_OFFSETS[field_name]
        data[offset:offset + 4] = struct.pack(">f", new_value)

    packet = build_packet(bytes(data))
    ser.write(packet)
    print(f"Sent SET_MCCONF: {len(packet)} bytes, changed {list(changes.keys())}")

    time.sleep(wait)
    ser.reset_input_buffer()
    return read_mcconf(ser)


def prompt_mcconf_changes(current):
    print("\nCurrent motor config values:")
    for name in MCCONF_OFFSETS:
        print(f"  {name}: {get_mcconf_field(current, name)}")

    print("\nEnter new values (press Enter to keep current):")
    changes = {}
    for name in MCCONF_OFFSETS:
        raw = input(f"  {name}: ").strip()
        if raw:
            try:
                changes[name] = float(raw)
            except ValueError:
                print(f"    '{raw}' is not a number, skipping {name}.")
    return changes


# --- Run ---

port_name = find_port()
if port_name is None:
    print("CH340 is not found")
else:
    ser = serial.Serial(port_name, baud_rate, timeout=1)
    print(f"Connected on {port_name}")

    current_app = read_appconf(ser)
    if current_app is not None:
        print(f"\nController ID: {current_app[5]}")
        print(f"CAN Baud Rate: {CAN_BAUD_LABELS.get(current_app[17], 'unknown')}")

    current_mc = read_mcconf(ser)
    if current_mc is not None:
        changes = prompt_mcconf_changes(current_mc)
        if changes:
            verify = write_mcconf(ser, current_mc, changes)
            if verify:
                print("\nVerified values:")
                for name in changes:
                    print(f"  {name}: {get_mcconf_field(verify, name)}")
        else:
            print("\nNo changes entered.")