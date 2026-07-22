"""
Step 1b: confirm the servo is actually responding to OUR specific
register read requests (not just background telemetry).

We send read requests for a few different registers, one at a time,
with a quiet gap before/after each, and print everything that comes
back. If a frame containing that exact register byte shows up right
after each request, that confirms we've found the real response.
"""

from urllib import response

import serial
import time

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0

# A handful of the registers the real app polls. Mix of values so we
# can clearly tell them apart in the output.
REGISTERS_TO_TEST = [0x6a, 0xce, 0x40, 0x2e]


def build_read_request(register: int, seq: int) -> bytes:
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
    body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)

def parse_register_response(data: bytes, register: int):
    """Search raw bytes for a register-read response frame matching
    `register`, and return its decoded value, or None if not found."""
    marker = bytes([0x04, 0x15, 0xf3, 0x03, 0x88, 0x0b, register])
    idx = data.find(marker)
    if idx == -1:
        return None
    val_lo = data[idx + 7]
    val_hi = data[idx + 8]
    return val_lo | (val_hi << 8)   # combine into a 16-bit value


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    # Clear out anything already buffered so each test starts clean.
    ser.reset_input_buffer()

    seq = 0xd2
    for register in REGISTERS_TO_TEST:
        frame = build_read_request(register, seq)
        seq += 1

        print(f"\n=== Requesting register 0x{register:02x} ===")
        print("Sent:", frame.hex(" "))

        ser.write(frame)
        response = b""
        deadline = time.time() + 1.0
        while time.time() < deadline:
            response += ser.read(64)

        print(f"Received ({len(response)} bytes):", response.hex(" "))

        value = parse_register_response(response, register)
        if value is not None:
            print(f"  -> register 0x{register:02x} = {value}")
        else:
            print(f"  -> no response found for 0x{register:02x}")

        time.sleep(1.0)  # quiet gap before the next request

    ser.close()


if __name__ == "__main__":
    main()