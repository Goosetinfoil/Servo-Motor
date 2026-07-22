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

    print("Calibration point collector. Holding value 0 (center).")
    print("Type a raw value; servo moves and holds. Note the angle. 'q' quits.\n")

    try:
        while True:
            raw = input("raw value> ").strip().lower()
            if raw in ("q", "quit", "exit"):
                break
            try:
                value = int(raw, 0)
            except ValueError:
                print("  enter an integer like 12000 or -8000")
                continue
            with lock:
                state["value"] = value
            print(f"  holding {value}. Read the angle and note: {value} -> ___ deg")
    finally:
        with lock:
            state["running"] = False
        thread.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    main()