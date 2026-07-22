import threading
import time

import serial
import serial.tools.list_ports

TARGET_VID = 0x10C4
TARGET_PID = 0xEA60
BAUD = 115200
TIMEOUT = 0.1

ID_REGISTERS = {"ID1 (Actuator ID)": 0x32, "ID2 (CAN Node ID)": 0x3E}
REPLY_MARKER = bytes([0xf3, 0x03, 0x88])


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


def build_read_request(register, seq):
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
    body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    return bytes(header + body + [checksum, 0x0a, 0x03])


def find_reply(response, register):
    search_from = 0
    while True:
        idx = response.find(REPLY_MARKER, search_from)
        if idx == -1 or idx + 7 > len(response):
            return None
        if response[idx + 4] == register:
            lo, hi = response[idx + 5], response[idx + 6]
            return lo | (hi << 8)
        search_from = idx + 1


def read_register(ser, register, seq, listen_time=2.0):
    frame = build_read_request(register, seq)
    ser.reset_input_buffer()
    ser.write(frame)
    collected = b""
    start = time.time()
    while time.time() - start < listen_time:
        chunk = ser.read(256)
        if chunk:
            collected += chunk
            val = find_reply(collected, register)
            if val is not None:
                return val
    print(f"[thread] read 0x{register:02x} failed. received: {collected.hex(' ')}")
    return None


def worker():
    print("[thread] waiting for servo...")
    port = None
    while port is None:
        port = find_servo_port()
        if port is None:
            time.sleep(1.0)
    print(f"[thread] found on {port}, connecting...")

    ser = serial.Serial(port, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)
    print("[thread] initializing...")
    initialize(ser)
    print("[thread] init done, reading IDs...")

    seq = 0xe0
    for label, address in ID_REGISTERS.items():
        val = read_register(ser, address, seq)
        seq = (seq + 1) & 0xFF
        print(f"[thread] {label}: {val}")
    ser.close()
    print("[thread] done.")


if __name__ == "__main__":
    t = threading.Thread(target=worker, daemon=True)
    t.start()
    t.join()