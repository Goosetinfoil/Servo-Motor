import time
import serial
import serial.tools.list_ports

TARGET_VID = 0x10C4
TARGET_PID = 0xEA60
BAUD = 115200
TIMEOUT = 0.1


def find_servo_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == TARGET_VID and p.pid == TARGET_PID:
            return p.device
    return None


def initialize(ser):
    seq = 0xc0
    for _ in range(5):
        ser.write(bytes([0x02, 0x53, 0x00, 0xff, 0x52, 0x03, 0x03])); time.sleep(0.03)
    for _ in range(11):
        ser.write(bytes([0x02, 0x58, 0x58, 0x03])); time.sleep(0.03)
    ser.write(bytes([0x3a, 0x41, 0x3a, 0x41, 0x3a, 0x41])); time.sleep(0.05)
    ser.write(bytes([0x02, 0x56, 0x53, 0x00, 0x00, 0x00, 0x00, 0x03])); time.sleep(0.15)
    ser.reset_input_buffer()

    def probe_reg(register):
        nonlocal seq
        for _ in range(2):
            body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
            hdr = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
            chk = sum(hdr[1:] + body) & 0xFF
            ser.write(bytes(hdr + body + [chk, 0x0a, 0x03]))
            seq = (seq + 1) & 0xFF
            time.sleep(0.02)
        tailB = {0xfc: 0x50, 0xfe: 0x54, 0x74: 0x40}[register]
        ser.write(bytes([0x02, 0x42, 0x00, 0x00, 0x00, 0x80, 0x96, 0x00,
                         register & 0xFF, 0x00, register & 0xFF, tailB, 0x0a, 0x03]))
        time.sleep(0.02)
        tailC = {0xfc: [0xcf, 0x08, 0x03], 0xfe: [0xd3, 0x08, 0x03],
                 0x74: [0xbf, 0x08, 0x03]}[register]
        ser.write(bytes([0x02, 0x41, 0x00, 0x00, 0x96, 0x00,
                         register & 0xFF, 0x00, register & 0xFF] + tailC))
        time.sleep(0.02)

    for n in range(4):
        activate = bytes([0x02, 0x53, n, 0x01, (n + 0x54) & 0xFF, 0x03, 0x03])
        for _ in range(5):
            ser.write(activate); time.sleep(0.03)
        for reg in (0xfc, 0xfe, 0x74):
            probe_reg(reg)
    time.sleep(0.3)
    ser.reset_input_buffer()


def main():
    port = find_servo_port()
    if not port:
        print("DPC-CAN not found. Plug it in.")
        return
    print(f"Using {port}")

    ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)
    print("Initializing (wake-up only, no read requests)...")
    initialize(ser)
    print("Init done. Listening passively for 5 seconds...\n")

    collected = b""
    start = time.time()
    while time.time() - start < 5.0:
        chunk = ser.read(256)
        if chunk:
            collected += chunk
    ser.close()

    print(f"Total bytes received: {len(collected)}")
    if collected:
        print("Raw dump:")
        print(collected.hex(" "))
    else:
        print("Nothing received at all after init.")

    print()
    if len(collected) <= 5:
        print("VERDICT: only the DPC-CAN ack (or nothing) came back.")
        print("The servo itself does not appear to be streaming after init.")
    else:
        print("VERDICT: the servo IS streaming data after init.")
        print("The wake-up is working - a separate problem with read requests.")


if __name__ == "__main__":
    main()