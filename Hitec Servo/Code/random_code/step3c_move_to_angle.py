"""
Step 3 (final): command the servo to an absolute ANGLE in degrees,
using the calibration read directly from the Hitec app.

Calibration (from the app's Left / Center / Right readouts):
    raw 1365   = -150.01 deg   (left extreme)
    raw 8192   =    0.00 deg    (center)
    raw 15018  = +149.99 deg   (right extreme)

This is a clean linear map:
    raw_count = CENTER_COUNT + degrees * COUNTS_PER_DEGREE

The raw count is an UNSIGNED value (range ~91..16300, center 8192 -
NOT the signed +-32767 scale we first assumed) and goes into bytes
8-9 of the move frame, little-endian:

    02 42 00 f2 03 80 00 00 <RAW_LO> <RAW_HI> <SEQ> <CHK> 0a 03

The servo holds position only while it's being commanded, so we stream
the current target continuously in a background thread, exactly like
the app does. Type an angle any time to retarget. 'q' to quit.

SAFETY: travel is about -150 deg .. +150 deg. Values are clamped to the
firmware limits. Retargeting across the range moves at full speed -
keep the servo clear of obstructions. Ctrl+C or 'q' stops the stream.
"""

import serial
import time
import threading

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0
SEND_INTERVAL = 0.03      # ~30 ms between frames, matching the app

# --- Calibration constants ---
CENTER_COUNT = 8192       # raw count at 0 degrees
COUNTS_PER_DEGREE = 45.51 # from the three-point app calibration

# Firmware position limits (from the app's MIN/MAX_LIMIT fields), used
# to clamp so we never command past the mechanical range.
MIN_COUNT = 91
MAX_COUNT = 16300

# Corresponding angle limits, derived from the count limits.
MIN_ANGLE = (MIN_COUNT - CENTER_COUNT) / COUNTS_PER_DEGREE   # ~ -178 deg
MAX_ANGLE = (MAX_COUNT - CENTER_COUNT) / COUNTS_PER_DEGREE   # ~ +178 deg


def angle_to_count(degrees: float) -> int:
    """Convert an angle in degrees to a raw position count, clamped to
    the servo's firmware limits."""
    count = round(CENTER_COUNT + degrees * COUNTS_PER_DEGREE)
    # Clamp into the safe mechanical range.
    return max(MIN_COUNT, min(MAX_COUNT, count))


def build_move_frame(degrees: float, seq: int) -> bytes:
    """Build a move frame that commands the given angle."""
    count = angle_to_count(degrees)

    # Raw count goes in unsigned, little-endian, into bytes 8 and 9.
    u16 = count & 0xFFFF
    lo = u16 & 0xFF
    hi = (u16 >> 8) & 0xFF

    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00, 0x00]
    body = [lo, hi, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF   # sum(bytes[1:11]) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


# Shared state between the input loop and the streaming thread.
state = {"angle": 0.0, "running": True}
lock = threading.Lock()


def stream_loop(ser: serial.Serial):
    """Background thread: continuously send the current target angle."""
    seq = 0x50
    while True:
        with lock:
            if not state["running"]:
                break
            angle = state["angle"]
        try:
            ser.write(build_move_frame(angle, seq))
        except serial.SerialException:
            break
        seq = (seq + 1) & 0xFF
        time.sleep(SEND_INTERVAL)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    thread = threading.Thread(target=stream_loop, args=(ser,), daemon=True)
    thread.start()

    print("Servo angle control. Enter an angle in degrees.")
    print(f"Range about {MIN_ANGLE:.0f} to {MAX_ANGLE:.0f} deg. 0 = center.")
    print("Try 0, then 45, then -45, then 90.  'q' to quit.\n")
    print("Currently holding 0 deg (center).\n")

    try:
        while True:
            raw = input("angle (deg)> ").strip().lower()
            if raw in ("q", "quit", "exit"):
                break
            try:
                angle = float(raw)
            except ValueError:
                print("  enter a number of degrees, e.g. 45 or -90")
                continue

            clamped = max(MIN_ANGLE, min(MAX_ANGLE, angle))
            with lock:
                state["angle"] = clamped
            count = angle_to_count(clamped)
            note = "" if clamped == angle else f" (clamped from {angle:g})"
            print(f"  -> {clamped:+.2f} deg  = raw count {count}{note}")
    finally:
        with lock:
            state["running"] = False
        thread.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    main()