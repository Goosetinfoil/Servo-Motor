import serial
import time

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0


def initialize(ser: serial.Serial):
    """Send the DPC-CAN startup sequence to wake it from dormant state."""
    print("Initializing DPC-CAN...")

    # Phase 1: probe (x5)
    for _ in range(5):
        ser.write(bytes([0x02, 0x53, 0x00, 0xff, 0x52, 0x03, 0x03]))
        time.sleep(0.05)

    # Phase 2: flush (x11)
    for _ in range(11):
        ser.write(bytes([0x02, 0x58, 0x58, 0x03]))
        time.sleep(0.05)

    # Phase 3: wake - triggers the 04 22 18 response from the DPC-CAN
    ser.write(bytes([0x3a, 0x41, 0x3a, 0x41, 0x3a, 0x41]))
    time.sleep(0.05)
    ser.write(bytes([0x02, 0x56, 0x53, 0x00, 0x00, 0x00, 0x00, 0x03]))
    time.sleep(0.2)  # wait for 04 22 18 response before continuing
    ser.reset_input_buffer()

    # Phase 4: activate - four incrementing frames, x5 each.
    # Telemetry only starts flowing after the N=03 frame is sent.
    for n in range(4):
        frame = bytes([0x02, 0x53, n, 0x01, (n + 0x54) & 0xFF, 0x03, 0x03])
        for _ in range(5):
            ser.write(frame)
            time.sleep(0.05)

    # Give the DPC-CAN a moment to start relaying telemetry.
    time.sleep(0.5)
    ser.reset_input_buffer()
    print("Initialization done.\n")


def build_read_request(register: int, seq: int) -> bytes:
    """Build a 14-byte read-register request frame."""
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
    body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


def listen(ser: serial.Serial, duration: float, label: str):
    """Listen on the port for `duration` seconds and print what arrives."""
    print(f"\n--- {label}: listening for {duration}s ---")
    start = time.time()
    collected = b""
    while time.time() - start < duration:
        chunk = ser.read(256)
        if chunk:
            collected += chunk
    print(f"Bytes received: {len(collected)}")
    if collected:
        print("First bytes:", collected[:128].hex(" "))
    else:
        print("Nothing received.")
    return collected


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    # Wake the DPC-CAN before doing anything else.
    initialize(ser)

    # Part 1: passive listen - telemetry should be flowing now.
    listen(ser, duration=3.0, label="Passive listen (no request sent)")

    # Part 2: send a read request and check for a response.
    frame = build_read_request(register=0x6a, seq=0xd2)
    print("\nSending read request:", frame.hex(" "))
    ser.write(frame)
    listen(ser, duration=2.0, label="After sending read request")

    ser.close()


if __name__ == "__main__":
    main()