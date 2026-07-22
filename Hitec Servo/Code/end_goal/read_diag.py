import time
import serial
import serial.tools.list_ports

TARGET_VID = 0x10C4
TARGET_PID = 0xEA60
BAUD = 115200


def find_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == TARGET_VID and p.pid == TARGET_PID:
            return p.device
    return None


def initialize(ser):
    """Full DPC-CAN + Auto Scan handshake, replicated byte-for-byte from
    the app's real startup capture (startup.html). This is the version
    that makes the servo actually respond to register reads - our earlier
    shorter init only woke the DPC-CAN, leaving the servo half-asleep.

    The app's Auto Scan cycles through all four activate levels N=0..3.
    At each level it sends the activate frame (x5) then probes the
    version/product registers (0xfc, 0xfe, 0x74) using three frame
    formats. Doing this whole sweep is what fully brings the servo online.
    """
    seq = 0xc0  # rolling sequence counter, matching the app's range

    # Phase 1: probe x5
    for _ in range(5):
        ser.write(bytes([0x02, 0x53, 0x00, 0xff, 0x52, 0x03, 0x03]))
        time.sleep(0.03)
    # Phase 2: flush x11
    for _ in range(11):
        ser.write(bytes([0x02, 0x58, 0x58, 0x03]))
        time.sleep(0.03)
    # Phase 3: wake
    ser.write(bytes([0x3a, 0x41, 0x3a, 0x41, 0x3a, 0x41]))
    time.sleep(0.05)
    ser.write(bytes([0x02, 0x56, 0x53, 0x00, 0x00, 0x00, 0x00, 0x03]))
    time.sleep(0.15)
    ser.reset_input_buffer()

    # Phase 4: Auto Scan - four activate levels, each followed by version
    # probes. This is the key part our old init was missing.
    def probe_reg(register):
        nonlocal seq
        # format A: standard read request, sent twice (as the app does)
        for _ in range(2):
            body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
            hdr = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
            chk = sum(hdr[1:] + body) & 0xFF
            ser.write(bytes(hdr + body + [chk, 0x0a, 0x03]))
            seq = (seq + 1) & 0xFF
            time.sleep(0.02)
        # format B: old-style 0x96 read, sent once. The byte after the
        # address is the register value echoed, then a fixed tail byte.
        tailB = {0xfc: 0x50, 0xfe: 0x54, 0x74: 0x40}[register]
        b = [0x02, 0x42, 0x00, 0x00, 0x00, 0x80, 0x96, 0x00,
             register & 0xFF, 0x00, register & 0xFF, tailB, 0x0a, 0x03]
        ser.write(bytes(b))
        time.sleep(0.02)
        # format C: 0x41 header read, sent once
        tailC = {0xfc: [0xcf, 0x08, 0x03], 0xfe: [0xd3, 0x08, 0x03],
                 0x74: [0xbf, 0x08, 0x03]}[register]
        c = [0x02, 0x41, 0x00, 0x00, 0x96, 0x00,
             register & 0xFF, 0x00, register & 0xFF] + tailC
        ser.write(bytes(c))
        time.sleep(0.02)

    for n in range(4):
        activate = bytes([0x02, 0x53, n, 0x01, (n + 0x54) & 0xFF, 0x03, 0x03])
        for _ in range(5):
            ser.write(activate)
            time.sleep(0.03)
        for reg in (0xfc, 0xfe, 0x74):
            probe_reg(reg)

    time.sleep(0.3)
    ser.reset_input_buffer()


def build_read_request(register, seq):
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
    body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    return bytes(header + body + [checksum, 0x0a, 0x03])


def main():
    port = find_port()
    if not port:
        print("DPC-CAN not found. Plug it in.")
        return
    print(f"Using {port}")

    ser = serial.Serial(port, BAUD, timeout=0.1)
    time.sleep(0.2)
    print("Initializing...")
    initialize(ser)
    print("Done.\n")

    REG = 0x32
    marker = bytes([0xf3, 0x03, 0x88])

    for trial in range(3):
        print(f"===== Trial {trial+1}: read register 0x{REG:02x} =====")
        frame = build_read_request(REG, 0xd2 + trial)
        ser.reset_input_buffer()
        print(f"Sending: {frame.hex(' ')}")
        ser.write(frame)

        # Listen 2 seconds, record every chunk with a timestamp.
        collected = b""
        t0 = time.time()
        found_at = None
        while time.time() - t0 < 2.0:
            chunk = ser.read(256)
            if chunk:
                collected += chunk
                idx = collected.find(marker)
                while idx != -1 and idx + 7 <= len(collected):
                    if collected[idx + 4] == REG:
                        lo, hi = collected[idx + 5], collected[idx + 6]
                        found_at = time.time() - t0
                        val = lo | (hi << 8)
                        print(f"  REPLY FOUND at {found_at:.3f}s: value = {val}")
                        break
                    idx = collected.find(marker, idx + 1)
                if found_at:
                    break

        print(f"  total bytes received: {len(collected)}")
        if not found_at:
            print("  NO reply for 0x32. Raw dump:")
            print("  " + collected.hex(" "))
        print()

    ser.close()


if __name__ == "__main__":
    main()