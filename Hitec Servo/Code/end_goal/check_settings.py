import serial
import time

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0

REGISTERS = {
    "dead_band": 0x4E,   # REG_DEADBAND
    "max_angle": 0x50,   # REG_POS_EMG_MAX
    "min_angle": 0x52,   # REG_POS_EMG_MIN
}

UNITS_PER_90_DEG = 4096  # per the manual, for all three registers above


def initialize(ser: serial.Serial):
    print("Initializing DPC-CAN...")
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
    print("  -> done.\n")


def build_read_request(register: int, seq: int) -> bytes:
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
    body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


def read_register(ser: serial.Serial, register: int, seq: int):
    """Send a read request and parse the little-endian value back out.
    Returns the raw unsigned 16-bit value, or None if unparseable."""
    frame = build_read_request(register, seq)
    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.3)
    response = ser.read(256)

    if bytes([register]) not in response:
        return None, response

    idx = response.find(bytes([register]))
    if idx + 2 >= len(response):
        return None, response

    lo, hi = response[idx + 1], response[idx + 2]
    value = lo | (hi << 8)
    return value, response


def to_signed16(value: int) -> int:
    """Reinterpret a raw unsigned 16-bit value as signed, in case a
    negative number got wrapped in during a write."""
    return value - 0x10000 if value >= 0x8000 else value


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    initialize(ser)

    print("Reading back current register values...\n")
    seq = 0xe0
    for name, address in REGISTERS.items():
        value, raw_response = read_register(ser, address, seq)
        seq = (seq + 1) & 0xFF

        print(f"{name} (register 0x{address:02x}):")
        if value is None:
            print(f"  Could not parse a value. Raw response: {raw_response.hex(' ')}")
        else:
            signed = to_signed16(value)
            degrees = signed / UNITS_PER_90_DEG * 90
            print(f"  raw = {value}  (0x{value:04x})")
            print(f"  as signed = {signed}")
            print(f"  as degrees (4096=90deg) = {degrees:.2f} deg")
        print()

    ser.close()


if __name__ == "__main__":
    main()