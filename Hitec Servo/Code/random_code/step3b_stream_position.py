"""
Step 3b (corrected): command position by CONTINUOUSLY streaming the
move frame, the way the Hitec app does while the slider is active.

Decoded move frame (verified against the slider capture):
  02 42 00 f2 03 80 00 00 <POS_LO> <POS_HI> <SEQ> <CHK> 0a 03

  byte 6      = 0x00            -> "this is a move" flag
  byte 7      = 0x00            -> constant
  bytes 8-9   = signed 16-bit   -> position, little-endian (byte8 low,
                                   byte9 high)
                                   0      = center (0.0)
                                   +32767 = full right (+1.0)
                                   -32767 = full left  (-1.0)
  byte 10     = sequence counter (rolling; servo is lenient about it)
  byte 11     = checksum = sum(bytes[1:11]) & 0xFF

The servo expects a continuous stream of these frames, not one-shot
commands, so this script streams the current target nonstop in a
background thread. Type a new position any time to retarget. 'q' quits.

SAFETY: keep the servo clear. Retargeting across the range moves at
full speed. Start near center and step outward gently the first time.
"""

import serial
import time
import threading

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0
SEND_INTERVAL = 0.03   # ~30 ms between frames, similar to the app's rate


def build_move_to(position_norm: float, seq: int) -> bytes:
    """Build a move frame for a normalized -1.0..+1.0 position."""
    # Clamp so we never command past the extremes.
    position_norm = max(-1.0, min(1.0, position_norm))

    # Scale the float to the signed 16-bit range the servo uses.
    pos = int(round(position_norm * 32767))

    # Turn the signed value into two little-endian bytes. (& 0xFFFF
    # converts a negative number to 16-bit two's-complement form.)
    pos_u16 = pos & 0xFFFF
    pos_lo = pos_u16 & 0xFF
    pos_hi = (pos_u16 >> 8) & 0xFF

    # header bytes 0-7: byte 6 (0x00) = move flag, byte 7 (0x00) = const.
    # Position low/high then land in bytes 8 and 9.
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00, 0x00]
    body = [pos_lo, pos_hi, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


# Shared state between the typing loop and the streaming thread.
state = {
    "target": 0.0,    # current commanded position
    "running": True,  # set False to stop the stream and exit
}
state_lock = threading.Lock()


def stream_loop(ser: serial.Serial):
    """Background thread: continuously send the current target position."""
    seq = 0x50
    while True:
        with state_lock:
            if not state["running"]:
                break
            target = state["target"]

        frame = build_move_to(target, seq)
        seq = (seq + 1) & 0xFF
        try:
            ser.write(frame)
        except serial.SerialException:
            break
        time.sleep(SEND_INTERVAL)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    # Start streaming. daemon=True so it won't keep the program alive
    # on its own if the main thread exits unexpectedly.
    thread = threading.Thread(target=stream_loop, args=(ser,), daemon=True)
    thread.start()

    print("Streaming position continuously. Enter -1.0..1.0 to retarget.")
    print("Starts at 0 (center). Try 0.3, then -0.3, then outward.  'q' to quit.\n")

    try:
        while True:
            raw = input("position> ").strip().lower()
            if raw in ("q", "quit", "exit"):
                break
            try:
                position = float(raw)
            except ValueError:
                print("  enter a number like 0.5 or -1.0")
                continue

            with state_lock:
                state["target"] = max(-1.0, min(1.0, position))
            print(f"  now streaming target {state['target']:+.3f}")
    finally:
        # Stop the thread cleanly and release the port before exiting.
        with state_lock:
            state["running"] = False
        thread.join(timeout=1.0)
        ser.close()


if __name__ == "__main__":
    main()