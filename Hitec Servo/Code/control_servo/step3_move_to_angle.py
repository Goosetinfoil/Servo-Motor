import serial
import time
import threading

PORT = "COM3"            # check Device Manager if this changed after replug
BAUD = 115200
TIMEOUT = 1.0
SEND_INTERVAL = 0.03

# --- Calibration (wire scale, center = 0) ---
COUNTS_PER_DEGREE = 45.51
MAX_ANGLE = 150.0
MIN_ANGLE = -150.0


def angle_to_wire(degrees: float) -> int:
    """Convert an angle in degrees to a signed wire value, clamped."""
    degrees = max(MIN_ANGLE, min(MAX_ANGLE, degrees))
    return round(degrees * COUNTS_PER_DEGREE)


def build_move_frame(degrees: float, seq: int) -> bytes:
    """Build a move frame commanding the given angle."""
    wire = angle_to_wire(degrees)

    # Signed value -> two little-endian bytes (& 0xFFFF gives the
    # 16-bit two's-complement form for negatives).
    u16 = wire & 0xFFFF
    lo = u16 & 0xFF
    hi = (u16 >> 8) & 0xFF

    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00, 0x00]
    body = [lo, hi, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


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

    print("Servo angle control. Enter an angle in degrees (-150 to 150).")
    print("0 = center. Try 0, 45, -45, 90.  'q' to quit.\n")
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
            wire = angle_to_wire(clamped)
            note = "" if clamped == angle else f" (clamped from {angle:g})"
            print(f"  -> {clamped:+.2f} deg = wire value {wire}{note}")
    finally:
        with lock:
            state["running"] = False
        thread.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    main()