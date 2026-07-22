"""
Step 2 (first real movement test): send ONE single-step nudge command
and visually confirm which physical direction it produces.

This sends exactly one frame, then exits - no loops, no held keys,
no repeated commands. Watch the servo horn when you run this and
note which way it moves.

Run it once with DIRECTION = "left", note what happens, then change
DIRECTION to "right" and run again. Don't run both in the same pass -
we want one clean, observable result per execution.
"""

import serial
import time

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0

# Change this between runs: "left" or "right"
DIRECTION = "right"

# These are the two byte9 values we found in the captures. 0x3b showed
# up consistently in the held_left_key capture, 0xbb in helf_right_key.
DIRECTION_CODES = {
    "left": 0x3b,
    "right": 0xbb,
}


def build_move_request(magnitude: int, direction_code: int, seq: int) -> bytes:
    """
    Build a 14-byte move/nudge command frame.

    magnitude:      byte8 in the captures - we're using 0xff (the
                     smallest single-step value we observed) so this
                     is the gentlest possible test.
    direction_code:  byte9 - the value that differed between left and
                     right key presses in our captures.
    seq:             byte10 - a sequence counter. The real app keeps
                     this continuously incrementing, but the servo
                     doesn't appear to validate it strictly, so any
                     value here should be fine for a test.
    """
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00]  # 0x00 = write/move flag
    body = [0x00, magnitude & 0xFF, direction_code & 0xFF, seq & 0xFF]

    # Same checksum formula we verified on every frame type so far:
    # sum of bytes 1 through 10, kept to one byte.
    checksum = sum(header[1:] + body) & 0xFF

    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


def main():
    direction_code = DIRECTION_CODES[DIRECTION]

    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    frame = build_move_request(magnitude=0xff, direction_code=direction_code, seq=0x50)

    print(f"Sending a single '{DIRECTION}' nudge: {frame.hex(' ')}")
    print("Watch the servo now.")

    ser.write(frame)
    time.sleep(0.3)

    # Drain and show whatever comes back, just for visibility - we're
    # not parsing it yet, just confirming the port is still alive.
    response = ser.read(256)
    print(f"Got {len(response)} bytes back: {response.hex(' ')}")

    ser.close()


if __name__ == "__main__":
    main()