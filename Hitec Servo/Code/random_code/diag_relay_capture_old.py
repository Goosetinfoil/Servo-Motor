"""
Diagnostic: replay VERBATIM frames captured from the app during a real
slider movement - no reconstruction, no math, exact bytes off the wire.

Purpose: settle whether our move frames are encoded wrong, or whether
the problem is something about getting bytes onto the bus at all (e.g.
the DPC-CAN needing the app's session first).

  - If these real captured frames MOVE the servo -> our encoding has a
    specific bug and we compare to find it.
  - If even these real frames do NOTHING -> the issue isn't the frame
    content; it's transport/session, and we look there instead.

We stream each captured frame continuously for a few seconds (the app
streamed continuously, so we match that), pausing between them. Watch
the servo during each phase and note any movement.

SAFETY: keep the servo clear - these command real positions across a
wide range. Ctrl+C stops everything.
"""

import serial
import time

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0
SEND_INTERVAL = 0.03

# Exact frames lifted from slider_movement capture (unmodified):
CAPTURED = [
    ("near -0.89 (far one side)", bytes([0x02,0x42,0x00,0xf2,0x03,0x80,0x00,0x00,0x8d,0x8e,0xd5,0xa7,0x0a,0x03])),
    ("near +0.26 (toward center)", bytes([0x02,0x42,0x00,0xf2,0x03,0x80,0x00,0x00,0xd6,0x21,0xd6,0x84,0x0a,0x03])),
    ("near +0.47 (other side)",    bytes([0x02,0x42,0x00,0xf2,0x03,0x80,0x00,0x00,0xfe,0x3b,0xd2,0xc2,0x0a,0x03])),
]

STREAM_SECONDS = 3.0   # how long to stream each frame


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    for label, frame in CAPTURED:
        print(f"\n>>> Streaming {label} for {STREAM_SECONDS}s")
        print(f"    frame: {frame.hex(' ')}")
        print("    WATCH THE SERVO NOW")
        end = time.time() + STREAM_SECONDS
        while time.time() < end:
            ser.write(frame)
            time.sleep(SEND_INTERVAL)
        print("    done - did it move?")
        time.sleep(1.0)  # pause so movements between phases are distinct

    ser.close()
    print("\nFinished. Report which phases (if any) produced movement.")


if __name__ == "__main__":
    main()