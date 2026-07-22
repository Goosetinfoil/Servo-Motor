import serial
import time

PORT = "COM3"
BAUD = 115200
TIMEOUT = 1.0

# A handful of the registers the real app polls. Mix of values so we
# can clearly tell them apart in the output.
REGISTERS_TO_TEST = [0x6a, 0xce, 0x40, 0x2e]


def build_read_request(register: int, seq: int) -> bytes:
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
    body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
    checksum = sum(header[1:] + body) & 0xFF
    footer = [0x0a, 0x03]
    return bytes(header + body + [checksum] + footer)


def main():
    ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
    time.sleep(0.2)

    # Clear out anything already buffered so each test starts clean.
    ser.reset_input_buffer()

    seq = 0xd2
    for register in REGISTERS_TO_TEST:
        frame = build_read_request(register, seq)
        seq += 1

        print(f"\n=== Requesting register 0x{register:02x} ===")
        print("Sent:", frame.hex(" "))

        ser.write(frame)
        time.sleep(0.3)  # short pause to let a response arrive

        response = ser.read(256)
        print(f"Received ({len(response)} bytes):", response.hex(" "))

        # Quick check: does the requested register byte show up
        # anywhere in the response?
        if bytes([register]) in response:
            print(f"  -> 0x{register:02x} FOUND in response")
        else:
            print(f"  -> 0x{register:02x} not found in response")

        time.sleep(1.0)  # quiet gap before the next request

    ser.close()


if __name__ == "__main__":
    main()