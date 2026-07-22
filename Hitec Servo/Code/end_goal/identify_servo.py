import serial
import time

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0

REG_ID1 = 0x32          # servo ID register, per manual sec 2-? (Comm)
REG_PRODUCT_NO = 0x74   # optional secondary check - product number

# If you've written down which ID corresponds to which physical servo,
# list them here so the script can print a friendly name instead of
# just a number. Fill this in as you assign IDs.
KNOWN_SERVOS = {
    # 1: "bottom",
    # 2: "left fin",
}


def initialize(ser: serial.Serial):
    """Send the DPC-CAN startup sequence to wake it from dormant state."""
    print("Step 1/3: Initializing DPC-CAN...")

    for _ in range(5):
        ser.write(bytes([0x02, 0x53, 0x00, 0xff, 0x52, 0x03, 0x03]))
        time.sleep(0.05)

    for _ in range(11):
        ser.write(bytes([0x02, 0x58, 0x58, 0x03]))
        time.sleep(0.05)

    ser.write(bytes([0x3a, 0x41, 0x3a, 0x41, 0x3a, 0x41]))
    time.sleep(0.05)
    ser.write(bytes([0x02, 0x56, 0x53, 0x00, 0x00, 0x00, 0x00, 0x03]))
    time.sleep(0.2)
    ser.reset_input_buffer()

    for n in range(4):
        frame = bytes([0x02, 0x53, n, 0x01, (n + 0x54) & 0xFF, 0x03, 0x03])
        for _ in range(5):
            ser.write(frame)
            time.sleep(0.05)

    time.sleep(0.5)
    ser.reset_input_buffer()
    print("  -> DPC-CAN initialized.\n")


def check_connection(ser: serial.Serial, duration: float = 2.0) -> bool:
    """Passively listen for telemetry to confirm the servo is talking."""
    print("Step 2/3: Checking servo connection...")
    start = time.time()
    collected = b""
    while time.time() - start < duration:
        chunk = ser.read(256)
        if chunk:
            collected += chunk

    if collected:
        print(f"  -> Connection OK ({len(collected)} bytes of telemetry received).\n")
        return True
    else:
        print("  -> No data received. Check the servo is plugged in and powered.\n")
        return False


def build_read_request(register: int, seq: int) -> bytes:
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
    body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


def read_register(ser: serial.Serial, register: int, seq: int, label: str):
    """Send a read request for `register` and return the raw response bytes."""
    frame = build_read_request(register, seq)
    ser.reset_input_buffer()
    ser.write(frame)
    time.sleep(0.3)
    response = ser.read(256)

    print(f"  Requested {label} (0x{register:02x})")

    if bytes([register]) in response:
        # The register byte echoes back in the response frame; the two
        # bytes immediately after it are the little-endian value.
        idx = response.find(bytes([register]))
        if idx + 2 < len(response):
            lo, hi = response[idx + 1], response[idx + 2]
            value = lo | (hi << 8)
            return value
    print(f"    -> could not parse a value for {label}")
    return None


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    initialize(ser)

    if not check_connection(ser):
        print("Stopping - fix the connection before trying to identify the servo.")
        ser.close()
        return

    print("Step 3/3: Identifying servo ID...")
    servo_id = read_register(ser, REG_ID1, seq=0xd2, label="REG_ID1")

    if servo_id is not None:
        name = KNOWN_SERVOS.get(servo_id)
        print(f"\n  -> Servo ID = {servo_id}")
    else:
        print("\n  -> Could not read servo ID. Try re-running, or check wiring.")

    ser.close()


if __name__ == "__main__":
    main()