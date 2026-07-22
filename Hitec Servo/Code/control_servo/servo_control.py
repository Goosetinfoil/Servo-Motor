import serial
import time
import threading
import bisect

PORT = "COM3"            # check Device Manager if this changed
BAUD = 115200
TIMEOUT = 1.0
SEND_INTERVAL = 0.03

DIRECTION_BIT = 0x8000   # bit 15 selects left
CEILING = 15360          # highest magnitude the servo accepts
DEADZONE_VALUE = 12500   # first value that produces motion (~10%)

# Calibration: (magnitude, angle_degrees), ascending.
CALIB = [
    (12500,  9.0),
    (13000, 13.5),
    (13500, 18.0),
    (14000, 27.0),
    (14500, 45.0),
    (15000, 63.0),
    (15360, 90.0),
]
MAX_ANGLE = CALIB[-1][1]   # 90


def angle_to_magnitude(angle_abs: float) -> int:
    """Convert an absolute angle (0..90) to a raw magnitude by
    interpolating the calibration table. Angles below the first
    calibration point map to the dead-zone value (center)."""
    angle_abs = max(0.0, min(MAX_ANGLE, angle_abs))

    angs = [a for m, a in CALIB]
    mags = [m for m, a in CALIB]

    # Below the first measured angle -> center (dead zone).
    if angle_abs <= angs[0]:
        # Scale linearly from 0 (=> value 0, center) up to the first
        # calibration point, so small angles still do something sensible.
        return round(angle_abs / angs[0] * mags[0])
    if angle_abs >= angs[-1]:
        return mags[-1]

    i = bisect.bisect_left(angs, angle_abs)
    a0, a1 = angs[i - 1], angs[i]
    m0, m1 = mags[i - 1], mags[i]
    frac = (angle_abs - a0) / (a1 - a0)
    return round(m0 + frac * (m1 - m0))


def build_move_frame(degrees: float, seq: int) -> bytes:
    """Build a move frame for a signed angle (-90 left .. +90 right)."""
    degrees = max(-MAX_ANGLE, min(MAX_ANGLE, degrees))

    magnitude = angle_to_magnitude(abs(degrees))
    magnitude = min(magnitude, CEILING)        # never exceed the ceiling

    value = magnitude & 0x7FFF
    if degrees < 0:                            # left: set direction bit
        value |= DIRECTION_BIT

    lo = value & 0xFF
    hi = (value >> 8) & 0xFF
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00, 0x00]
    body = [lo, hi, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    return bytes(header + body + [checksum, 0x0a, 0x03])


state = {"angle": 0.0, "running": True}
lock = threading.Lock()


def stream_loop(ser: serial.Serial):
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

    print("Servo angle control (full range).")
    print(f"Enter an angle from -{MAX_ANGLE:.0f} (left) to +{MAX_ANGLE:.0f} (right). 0 = center.")
    print("Try 0, 45, 90, then -45, -90.  'q' to quit.\n")

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

            clamped = max(-MAX_ANGLE, min(MAX_ANGLE, angle))
            with lock:
                state["angle"] = clamped
            mag = min(angle_to_magnitude(abs(clamped)), CEILING)
            side = "center" if clamped == 0 else ("left" if clamped < 0 else "right")
            note = "" if clamped == angle else f" (clamped from {angle:g})"
            print(f"  -> {clamped:+.1f} deg ({side}, magnitude {mag}){note}")
    finally:
        with lock:
            state["running"] = False
        thread.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    main()