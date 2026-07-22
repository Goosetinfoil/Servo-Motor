"""
Step 2b: produce VISIBLE motion by repeating the nudge command,
the same way holding the arrow key in the app does.

We confirmed a single nudge moves the servo only one encoder step
(too small to see). The app's held-key captures showed motion comes
from sending the frame many times in quick succession - NOT from a
large single value. So we replicate that: a bounded loop of repeated
nudges with a small delay, and we read back position as we go.

IMPORTANT: this is bounded (REPEATS is finite) and stoppable (Ctrl+C).
Keep the servo clear of obstructions and keep REPEATS modest at first.
"""

import serial
import time

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0

DIRECTION = "left"          # "left" or "right"
REPEATS = 100                # how many nudge frames to send - start small
DELAY_BETWEEN = 0.03        # seconds between frames (~ matches app's rate)

DIRECTION_CODES = {
    "left": 0x3b,
    "right": 0xbb,
}


def build_move_request(magnitude: int, direction_code: int, seq: int) -> bytes:
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00]
    body = [0x00, magnitude & 0xFF, direction_code & 0xFF, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


def read_position(data: bytes):
    """
    Pull the latest position-feedback frame out of a chunk of bytes.
    These look like: 04 15 f3 03 88 <p0> <p1> 0b <p2> <p3> 00 00 ...
    We return the 4 position-ish bytes so we can watch them change.
    Returns None if no such frame is found.
    """
    marker = bytes([0x04, 0x15, 0xf3, 0x03, 0x88, 0x9a])
    idx = data.rfind(marker)   # rfind = most recent one in the buffer
    if idx == -1 or idx + 10 > len(data):
        return None
    return data[idx + 5:idx + 7].hex(" ")


def main():
    direction_code = DIRECTION_CODES[DIRECTION]

    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)
    ser.reset_input_buffer()

    # Position before we start moving.
    before = ser.read(128)
    print("Position before:", read_position(before))

    print(f"Sending {REPEATS} '{DIRECTION}' nudges...")
    seq = 0x50
    for i in range(REPEATS):
        frame = build_move_request(magnitude=0xff,
                                   direction_code=direction_code,
                                   seq=seq)
        ser.write(frame)
        seq = (seq + 1) & 0xFF      # increment like the real app does
        time.sleep(DELAY_BETWEEN)

    # Let telemetry settle, then read position after.
    time.sleep(0.2)
    after = ser.read(256)
    print("Position after: ", read_position(after))

    ser.close()


if __name__ == "__main__":
    main()