import serial
import time
import threading

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0
SEND_INTERVAL = 0.03


def build_frame(raw_value: int, seq: int) -> bytes:
    u16 = raw_value & 0xFFFF
    lo, hi = u16 & 0xFF, (u16 >> 8) & 0xFF
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00, 0x00]
    body = [lo, hi, seq & 0xFF]
    chk = sum(header[1:] + body) & 0xFF
    return bytes(header + body + [chk, 0x0a, 0x03])


def parse_positions(data: bytes):
    """Pull candidate position fields from the most recent long telemetry
    frame (the 0c 05 type). Returns a dict of candidate interpretations."""
    marker = bytes([0x04, 0x15, 0xf3, 0x03, 0x88])
    idx = data.rfind(marker)
    if idx == -1 or idx + 12 > len(data):
        return None
    f = data[idx:idx + 12]
    # f = 04 15 f3 03 88 p0 p1 0b p2 p3 00 00
    p0, p1, p2, p3 = f[5], f[6], f[8], f[9]
    return {
        "raw": f[5:10].hex(" "),
        "[p0,p1]LE": p0 | (p1 << 8),
        "[p2,p3]LE": p2 | (p3 << 8),
    }


state = {"value": 0, "running": True}
lock = threading.Lock()


def stream_loop(ser):
    seq = 0x50
    while True:
        with lock:
            if not state["running"]:
                break
            value = state["value"]
        try:
            ser.write(build_frame(value, seq))
        except serial.SerialException:
            break
        seq = (seq + 1) & 0xFF
        time.sleep(SEND_INTERVAL)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    thread = threading.Thread(target=stream_loop, args=(ser,), daemon=True)
    thread.start()

    print("Calibration with read-back. Type a raw value, watch the reported")
    print("position fields settle. We want to find which field tracks input.")
    print("Try: 0, then 12288, then 15018, then 15358, then 8000, 4000.")
    print("'q' to quit.\n")

    try:
        while True:
            raw = input("raw value> ").strip().lower()
            if raw in ("q", "quit", "exit"):
                break
            try:
                value = int(raw, 0)
            except ValueError:
                print("  enter an integer like 12288 or 0x3000")
                continue
            with lock:
                state["value"] = value

            # Let it move, then sample position a few times.
            print(f"  commanded raw {value} (0x{value & 0xFFFF:04x}). Reading position...")
            for _ in range(3):
                time.sleep(0.5)
                ser.reset_input_buffer()
                time.sleep(0.2)
                data = ser.read(256)
                pos = parse_positions(data)
                if pos:
                    print(f"    reported: {pos['raw']}   "
                          f"[p0,p1]LE={pos['[p0,p1]LE']:5}   "
                          f"[p2,p3]LE={pos['[p2,p3]LE']:5}")
                else:
                    print("    (no position frame found)")
            print()
    finally:
        with lock:
            state["running"] = False
        thread.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    main()