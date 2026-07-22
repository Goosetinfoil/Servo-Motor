"""
Step 2c: interactive position prober.

Goal: build a value -> physical-position map by hand, safely.

You type a single byte-8 value (in hex, e.g. 00, 40, 7f, 80, c0, ff)
and this sends EXACTLY ONE frame with that value, then reads back the
servo's reported position. You watch where the horn parks, note it,
and try the next value. No loops, no sweeps - one deliberate frame at
a time, so nothing moves at full speed across the range unexpectedly.

Theory we're testing: byte 8 is an ABSOLUTE signed position, where
0x00 = center, 0x7f ~= +1.0 (one extreme), 0x80/0x81 ~= -1.0 (other
extreme). We want to confirm that with real observation.

SAFETY: keep the servo clear of obstructions. Start near where it
already is and step outward gradually - don't jump from one extreme
to the other on your first few tries until you know the mapping.
Type 'q' to quit.
"""

import serial
import time

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0

# Direction byte. We'll keep this fixed for now and vary only byte 8.
# 0x3b was the value seen in the "left" captures.
DIRECTION_CODE = 0x3b


def build_move_request(position: int, direction_code: int, seq: int) -> bytes:
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00]
    body = [0x00, position & 0xFF, direction_code & 0xFF, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


def read_position(data: bytes):
    """Return the live position bytes from the long feedback frame."""
    marker = bytes([0x04, 0x15, 0xf3, 0x03, 0x88, 0x9a])
    idx = data.rfind(marker)
    if idx == -1 or idx + 7 > len(data):
        # Fall back to any long frame if the 0x9a one isn't present -
        # the position byte after 0x88 may have changed.
        marker = bytes([0x04, 0x15, 0xf3, 0x03, 0x88])
        idx = data.rfind(marker)
        if idx == -1 or idx + 7 > len(data):
            return None
    return data[idx + 5:idx + 7].hex(" ")


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)
    ser.reset_input_buffer()

    seq = 0x50

    print("Interactive position prober. Enter a byte-8 value in hex.")
    print("Examples: 00 (center?), 40, 7f, 80, c0, ff.  Type 'q' to quit.\n")

    while True:
        raw = input("byte8 hex> ").strip().lower()
        if raw in ("q", "quit", "exit"):
            break
        try:
            position = int(raw, 16)
            if not (0 <= position <= 0xFF):
                print("  value must be 00-ff")
                continue
        except ValueError:
            print("  not valid hex, try again (e.g. 7f)")
            continue

        # Read position right before sending.
        ser.reset_input_buffer()
        before = ser.read(128)
        pos_before = read_position(before)

        frame = build_move_request(position, DIRECTION_CODE, seq)
        seq = (seq + 1) & 0xFF
        print(f"  sending: {frame.hex(' ')}")
        ser.write(frame)

        # Give it a moment to move and report back.
        time.sleep(0.4)
        after = ser.read(256)
        pos_after = read_position(after)

        print(f"  position before: {pos_before}   ->   after: {pos_after}")
        print(f"  (watch the horn - note where 0x{position:02x} parks it)\n")

    ser.close()


if __name__ == "__main__":
    main()