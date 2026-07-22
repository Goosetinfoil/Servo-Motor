"""
Step 3 (first version): command the servo to an ABSOLUTE position
using the now-decoded move frame.

Decoded move frame:
  02 42 00 f2 03 80 00 00 <POS_LO> <POS_HI> <SEQ> <CHK> 0a 03

  byte 6      = 0x00            -> "this is a move" flag
  bytes 7-8   = signed 16-bit   -> position, little-endian
                                   0      = center (0.0)
                                   +32767 = full right (+1.0)
                                   -32768 = full left  (-1.0)
  byte 10     = sequence counter (rolling, servo is lenient about it)
  byte 11     = checksum = sum(bytes[1:11]) & 0xFF

This is what we were missing: position is TWO bytes, not one. Our
earlier single-byte pokes only set the high byte and left the low
byte at 0, sending a malformed half-position the servo ignored.

You give it a float from -1.0 to +1.0 and it sends one clean frame.
"""

import serial
import time

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0


def build_move_to(position_norm: float, seq: int) -> bytes:
    """
    Build a move frame for a normalized position.

    position_norm: -1.0 (full left) .. 0.0 (center) .. +1.0 (full right)
    seq:           rolling sequence byte
    """
    # Clamp to the safe range so we never command past the extremes.
    position_norm = max(-1.0, min(1.0, position_norm))

    # Scale the -1.0..+1.0 float to the signed 16-bit range the servo
    # uses. We use 32767 so +1.0 -> 32767 and -1.0 -> -32767.
    pos = int(round(position_norm * 32767))

    # Convert the signed value into two little-endian bytes.
    # (& 0xFFFF turns a negative number into its 16-bit two's-complement
    # form; then we split low and high bytes.)
    pos_u16 = pos & 0xFFFF
    pos_lo = pos_u16 & 0xFF
    pos_hi = (pos_u16 >> 8) & 0xFF

    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00]  # 0x00 = move
    body = [pos_lo, pos_hi, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    print("Enter a position from -1.0 (left) to 1.0 (right). 'q' to quit.")
    print("Suggested first tries: 0 (center), then 0.5, then -0.5.\n")

    seq = 0x50
    while True:
        raw = input("position> ").strip().lower()
        if raw in ("q", "quit", "exit"):
            break
        try:
            position = float(raw)
        except ValueError:
            print("  enter a number like 0.5 or -1.0")
            continue

        frame = build_move_to(position, seq)
        seq = (seq + 1) & 0xFF
        print(f"  sending: {frame.hex(' ')}  (target {position:+.3f})")
        ser.write(frame)
        time.sleep(0.2)

    ser.close()


if __name__ == "__main__":
    main()