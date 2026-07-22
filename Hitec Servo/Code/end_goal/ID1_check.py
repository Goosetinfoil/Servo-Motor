import time
import serial
import serial.tools.list_ports

TARGET_VID = 0x10C4
TARGET_PID = 0xEA60
BAUD = 115200

# --- find the port ---
port = None
for p in serial.tools.list_ports.comports():
    if p.vid == TARGET_VID and p.pid == TARGET_PID:
        port = p.device
        break

if port is None:
    print("DPC-CAN not found. Plug it in.")
    raise SystemExit

print(f"Using {port}")
ser = serial.Serial(port, BAUD, timeout=0.1)
time.sleep(0.2)

# --- initialize (wake-up + Auto Scan) ---
seq = 0xc0

for _ in range(5):
    ser.write(bytes([0x02, 0x53, 0x00, 0xff, 0x52, 0x03, 0x03]))
    time.sleep(0.03)
for _ in range(11):
    ser.write(bytes([0x02, 0x58, 0x58, 0x03]))
    time.sleep(0.03)
ser.write(bytes([0x3a, 0x41, 0x3a, 0x41, 0x3a, 0x41]))
time.sleep(0.05)
ser.write(bytes([0x02, 0x56, 0x53, 0x00, 0x00, 0x00, 0x00, 0x03]))
time.sleep(0.15)
ser.reset_input_buffer()

for n in range(4):
    activate = bytes([0x02, 0x53, n, 0x01, (n + 0x54) & 0xFF, 0x03, 0x03])
    for _ in range(5):
        ser.write(activate)
        time.sleep(0.03)
    for register in (0xfc, 0xfe, 0x74):
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

time.sleep(0.3)
ser.reset_input_buffer()
print("Init done.")

# --- read ID1 (register 0x32) ---
REG = 0x32
marker = bytes([0xf3, 0x03, 0x88])

frame_seq = 0xd2
header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
body = [REG & 0xFF, 0x00, 0x00, frame_seq & 0xFF]
checksum = sum(header[1:] + body) & 0xFF
frame = bytes(header + body + [checksum, 0x0a, 0x03])

ser.reset_input_buffer()
print(f"Sending: {frame.hex(' ')}")
ser.write(frame)

collected = b""
t0 = time.time()
id1 = None
while time.time() - t0 < 3.0:
    chunk = ser.read(256)
    if chunk:
        collected += chunk
        idx = collected.find(marker)
        while idx != -1 and idx + 7 <= len(collected):
            if collected[idx + 4] == REG:
                lo, hi = collected[idx + 5], collected[idx + 6]
                id1 = lo | (hi << 8)
                break
            idx = collected.find(marker, idx + 1)
        if id1 is not None:
            break

# --- read ID2 (register 0x3E) ---
REG2 = 0x3E
frame_seq2 = 0xd3
body2 = [REG2 & 0xFF, 0x00, 0x00, frame_seq2 & 0xFF]
checksum2 = sum(header[1:] + body2) & 0xFF
frame2 = bytes(header + body2 + [checksum2, 0x0a, 0x03])

ser.reset_input_buffer()
print(f"Sending: {frame2.hex(' ')}")
ser.write(frame2)

collected2 = b""
t0 = time.time()
id2 = None
while time.time() - t0 < 3.0:
    chunk = ser.read(256)
    if chunk:
        collected2 += chunk
        idx = collected2.find(marker)
        while idx != -1 and idx + 7 <= len(collected2):
            if collected2[idx + 4] == REG2:
                lo, hi = collected2[idx + 5], collected2[idx + 6]
                id2 = lo | (hi << 8)
                break
            idx = collected2.find(marker, idx + 1)
        if id2 is not None:
            break

ser.close()

print()
if id1 is not None:
    print(f"ID1 = {id1}")
else:
    print("ID1 read failed.")
    print(f"Received ({len(collected)} bytes): {collected.hex(' ')}")

if id2 is not None:
    print(f"ID2 = {id2}")
else:
    print("ID2 read failed.")
    print(f"Received ({len(collected2)} bytes): {collected2.hex(' ')}")