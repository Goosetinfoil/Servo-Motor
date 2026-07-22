"""
Calibration prober: stream a RAW 16-bit value (no scale assumptions)
and observe where the servo physically parks, to build the real
value -> position mapping.

We confirmed verbatim frames move the servo, but the raw 16-bit value
does NOT map to position the way we assumed (a frame decoding to -0.886
moved RIGHT, not left). So we stop assuming and measure directly.

You enter a raw integer (decimal, e.g. 0, 8000, 16000, -16000, 30000)
or hex with 0x. It builds a valid frame (correct checksum) carrying
that value in bytes 8-9 and streams it continuously so the servo
actually holds. You watch where the horn points and record it.

Goal: enter several values and note physical position for each, so we
can work out the true formula. Suggested sweep below in the prompt.

SAFETY: keep the servo clear. Type 'q' to quit (streaming stops).
"""

import serial
import time
import threading

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0
SEND_INTERVAL = 0.03


def build_frame(raw_value: int, seq: int) -> bytes:
    """Put a raw 16-bit value into bytes 8-9 (little-endian) and
    compute the verified checksum. No scaling - raw_value goes in as-is."""
    u16 = raw_value & 0xFFFF
    lo = u16 & 0xFF
    hi = (u16 >> 8) & 0xFF
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00, 0x00]
    body = [lo, hi, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


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

    print("Raw 16-bit value prober. Enter a value, watch where it parks.")
    print("Suggested sweep to map the range:")
    print("   0, 4000, 8000, 12000, 16000, 20000, 24000, 28000, 32000")
    print("   then negatives: -4000, -8000, ... -32000")
    print("Decimal or 0x-hex. 'q' to quit.\n")
    print("Currently streaming value 0.\n")

    try:
        while True:
            raw = input("raw value> ").strip().lower()
            if raw in ("q", "quit", "exit"):
                break
            try:
                value = int(raw, 0)  # base 0 = accept decimal or 0x hex
            except ValueError:
                print("  enter an integer like 16000 or 0x3e80")
                continue
            with lock:
                state["value"] = value
            u16 = value & 0xFFFF
            print(f"  streaming raw {value} (bytes: {u16 & 0xFF:02x} {(u16>>8)&0xFF:02x}) - note where it points")
    finally:
        with lock:
            state["running"] = False
        thread.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    main()