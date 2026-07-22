import serial 
import time 

def calculate_red_checksum(servo_id, address):
    return (servo_id + address + 0) & 0xFF

def read_register(ser, servo_id, address):
    checksum = calculate_red_checksum(servo_id, address)

    packet = bytes([
        0x96,
        servo_id,
        address,
        0x00,
        checksum
    ])

    
    print(f"Sending packet: {packet.hex()}")
    ser.write(packet)

    response = ser.read(7)
    print(f"Received: {response.hex()}")

    if len(response) == 7 and response[0] == 0x69:
        data_low = response[4]
        data_high = response[5]
        value = data_low + (data_high *256)
        return value
    return None


if __name__ == "__main__":
    ser = serial.Serial(
        port='COM3',
        baudrate=115200,
        stopbits=serial.STOPBITS_ONE,
        parity=serial.PARITY_NONE,
        bytesize=serial.EIGHTBITS,
        timeout=2
    )

    print(f"Port Open: {ser.is_open}")

    time.sleep(2)

    ser.reset_input_buffer()
    ser.reset_output_buffer()

    print("Listening for startup messages...")
    time.sleep(2)
    if ser.in_waiting > 0:
        data = ser.read(ser.in_waiting)
        print(f"Startup data: {data.hex()}")
    else:
        print("No startup data")

    print("Reading servo version...")
    version = read_register(ser, 11, 0xFC)
    if version is not None:
        print(f"Servo Version: {version}")
    else:
        print("Failed to read servo version.")

    ser.close()
    print("Port Closed")