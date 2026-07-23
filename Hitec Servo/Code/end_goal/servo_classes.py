import time
import tkinter as tk
from tkinter import ttk
import serial
import serial.tools.list_ports
import struct


# Parent class
class Servo:
    def __init__(self, vid, pid, baud):
        """Stores the connection info for the two servo. Stored in a parent class"""
        self.vid = vid
        self.pid = pid
        self.baud = baud
        self.ser = None

    def find_port(self):
        """Scans for connected serial decvices on port for one 
        matching the specific servo connected. """
        for p in serial.tools.list_ports.comports():
            if p.vid == self.vid and p.pid == self.pid:
                return p.device
        return None
    

# ----------------------------------------
#  Hitec Servo
# ----------------------------------------

# Constants to find specific adapter chip (Silicon Labs CP210x)
HITEC_TARGET_VID = 0x10C4 
HITEC_TARGET_PID = 0xEA60

# Speed of USB connection between computor and the DPC-CAN adapter.
HITEC_BAUD = 115200 

class HitecServo(Servo):

    marker = bytes([0xf3, 0x03, 0x88]) # This is the unique byte sequence that appears in every response from the servo
    header = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80] # This is the unique byte sequence that appears at the start of every read request to the servo

    def __init__(self):
        super().__init__(HITEC_TARGET_VID, HITEC_TARGET_PID, HITEC_BAUD)
    
    def connect(self):
        port = self.find_port()
        if port is None:
            return False
        else:
            self.ser = serial.Serial(port, self.baud, timeout=0.1)
            time.sleep(0.2)
            self.wake_servo()
            return True
        
    def read_register(self, register, frame_seq, timeout=3.0): # Same concept as Format A
        """This function requests one register value from the servo and waits 
        for tis reply. `ser` must already be open and woken via wake_servo(); 
        `register` is the register address to read; `frame_seq` is the sequence 
        byte to stamp the request with (must be in the 0xd2-0xd7 range
        Returns the register's 16-bit unsigned value, or None if no valid reply 
        arrived within `timeout` seconds."""

        body = [register & 0xFF, 0x00, 0x00, frame_seq & 0xFF]
        checksum = sum(self.header[1:] + body) & 0xFF
        frame = bytes(self.header + body + [checksum, 0x0a, 0x03])

        self.ser.reset_input_buffer() # Clears any leftover bytes
        self.ser.write(frame) # Writes to the servo
    
        # Collects the response from the servo within 3 seconds and just stores it instead of looking through it byte by byte.
        collected = b""
        t0 = time.time()
        value = None
        while time.time() - t0 < timeout:
            chunk = self.ser.read(256) # Reads upto 256 bytes
            if chunk:
                collected += chunk
                idx = collected.find(self.marker) # After storing the response, it looks for the marker in the response and stores it as idx
                while idx != -1 and idx + 7 <= len(collected): # if the marker is found, and there are more than 7 bytes after the marker, it protects it
                    if collected[idx + 4] == register: # This is the resgister byte inside the reply. It compars this value to the register value from earlier
                        lo, hi = collected[idx + 5], collected[idx + 6]
                        value = lo | (hi << 8)
                        break
                    idx = collected.find(self.marker, idx + 1) # If it doesn't match, it looks for the next marker in the response and repeats the process. 
                if value is not None:
                    break
        return value
    
    # Initialize (wake-up + Auto Scan). This whole block is a byte-for-byte replay of what the official Hitec Configure App sends on startup.
    def wake_servo(self):
        """Brings the servo out of its sleep state so it will respond to register 
        reads and writes. Writes a fixed, timed sequence of bytes to it through a 
        open serial connection. Returns no value. This must be called after opening 
        opening the port and before reading or writing values."""

        seq = 0xc0

        for _ in range(5):
            self.ser.write(bytes([0x02, 0x53, 0x00, 0xff, 0x52, 0x03, 0x03])) # Sends these bytes to the servo
            time.sleep(0.03) 
        for _ in range(11): # Same as above loop, but with different bytes.
            self.ser.write(bytes([0x02, 0x58, 0x58, 0x03]))
            time.sleep(0.03)
        self.ser.write(bytes([0x3a, 0x41, 0x3a, 0x41, 0x3a, 0x41])) # This is a message to the DPC-CAN and not the servo
        time.sleep(0.05)
        self.ser.write(bytes([0x02, 0x56, 0x53, 0x00, 0x00, 0x00, 0x00, 0x03]))
        time.sleep(0.15)
        self.ser.reset_input_buffer()

        for n in range(4):
            activate = bytes([0x02, 0x53, n, 0x01, (n + 0x54) & 0xFF, 0x03, 0x03])
            for _ in range(5):
                self.ser.write(activate)
                time.sleep(0.03)
            # Format A - Message 1 to be sent
            for register in (0xfc, 0xfe, 0x74):# specific servo registers that the app check for(properly version control)
                for _ in range(2): # Happens twice(seq = 192 and seq = 193)
                    body = [register & 0xFF, 0x00, 0x00, seq & 0xFF]
                    hdr = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x80]
                    chk = sum(hdr[1:] + body) & 0xFF # Checksum(chk) is a way of checking if the data received matches the transmitted data
                    self.ser.write(bytes(hdr + body + [chk, 0x0a, 0x03]))
                    seq = (seq + 1) & 0xFF
                    time.sleep(0.02)
                # Format B - Message 2 to be sent
                tailB = {0xfc: 0x50, 0xfe: 0x54, 0x74: 0x40}[register]
                self.ser.write(bytes([0x02, 0x42, 0x00, 0x00, 0x00, 0x80, 0x96, 0x00, register & 0xFF, 0x00, register & 0xFF, tailB, 0x0a, 0x03]))
                time.sleep(0.02)
                # Format C - Messge 3 to be sent
                tailC = {0xfc: [0xcf, 0x08, 0x03], 0xfe: [0xd3, 0x08, 0x03], 0x74: [0xbf, 0x08, 0x03]}[register]
                self.ser.write(bytes([0x02, 0x41, 0x00, 0x00, 0x96, 0x00, register & 0xFF, 0x00, register & 0xFF] + tailC))
                time.sleep(0.02)

        time.sleep(0.3)
        self.ser.reset_input_buffer() # Clears any leftover bytes
        "At the end of this block the servo is awake and ready to respond to register reads/writes."

    # Reads ID1, ID2, Speed, Pos Max, Pos Min from the servo
    def read_all_data(self):
        """Opens the port, wakes the servo, reads all 12 known registers,
        then closes the port. Position values are converted from raw
        unsigned to signed via to_signed16(); everything else is left raw.
        Returns (data_dict, None) on success, or (None, error_message) if
        the adapter wasn't found."""
        port = self.connect()
        if port is False:
            return None, "DPC-CAN not found. Plug it in"
        
        # Reads the following values from the servo and stores them in variables.
        id1 = self.read_register(0x32, 0xd2)
        id2 = self.read_register(0x3E, 0xd3)
        speed = self.read_register(0x54, 0xd4)
        pos_max = self.read_register(0xB0, 0xd5)
        pos_min = self.read_register(0xB2, 0xd6)
        pos_mid = self.read_register(0xC2, 0xd7)
        baud_rate = self.read_register(0x38, 0xd8)
        sample_point = self.read_register(0x40, 0xd9)
        stream_time = self.read_register(0x2E, 0xda)
        stream_mode = self.read_register(0x30, 0xdb)
        speed_up = self.read_register(0xDC, 0xdc)
        speed_dn = self.read_register(0xDE, 0xdd)

        self.ser.close() # Closes the com port

        data = { # A dictionary literal where each "key" is a text label matching the same key names throughout the code and the app.
        # Each value is one of the numbers 'read_one' already fetched from the earlier function
            "id1": id1,
            "id2": id2,
            "speed": speed,
            "pos_max": HitecServo.to_signed16(pos_max),
            "pos_min": HitecServo.to_signed16(pos_min),
            "pos_mid": HitecServo.to_signed16(pos_mid),
            "baud_rate": baud_rate,
            "sample_point": sample_point,
            "stream_time": stream_time,
            "stream_mode": stream_mode,
            "speed_up": speed_up,
            "speed_dn": speed_dn,
        }
        return data, None
    
    # This whole block is a faster version of read_all_data() since it only reads ID1 and ID2
    def read_ids_only(self): 
        """Same open/wake/close cycle as read_all_data(), but only reads
        ID1 and ID2 - used by the 'Read ID' button, which needs to be quick
        and doesn't care about the other 10 registers. Returns
        ((id1, id2), None) on success, or (None, error_message) on failure."""
        # This is a exact copy of waking up the servo, looking for the bytes that represent ID1 and ID2 and storing the values. 
        port = self.connect()
        if port is False:
            return None, "DPC-CAN not found. Plug it in."

        id1 = self.read_register(0x32, 0xd2)
        id2 = self.read_register(0x3E, 0xd3)
        self.ser.close()
        return (id1, id2), None
    
    # This saves the values to the servo 
    def write_all(self, values):
        """Open the servo once, initialize once, write EVERY (register, value)
        pair in `values` (a dict of register -> value), send the save command
        once at the end, then read each register back to confirm.

        Returns (confirmed_dict, error_message_or_None) where confirmed_dict
        maps register -> the value the servo reports after saving (or None if
        that register's read-back failed)."""

        port = self.connect()
        if port is False:
            return None, "DPC-CAN not found. Plug it in."


        # --- write every register (confirmed write frame format), one after
        #     another, with a rolling sequence byte in the proven range ---

        wseq = 0xd4
        for register, value in values.items(): # values.items yields the register and value from the dict earlier to be upacked in the loop     
            u16 = value & 0xFFFF # Clamps value to 16 bits and converts a negative Python number into its correct unsigned wire representation. 
            lo, hi = u16 & 0xFF, (u16 >> 8) & 0xFF
                # Splits u16 into its low byte (lo — bottom 8 bits, no shift needed) and high byte (hi — shift the top 8 bits right by 8 positions, then mask)
            whdr = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00] # Same as read header except the last byte, 0x00 means "I am telling something"
            wbody = [register & 0xFF, lo, hi, wseq & 0xFF] # Same as read body except middle bytes. We are telling something so we place lo, hi
            wchk = sum(whdr[1:] + wbody) & 0xFF # Exact same checksum 
            self.ser.write(bytes(whdr + wbody + [wchk, 0x0a, 0x03])) # Exact same write 
            wseq = (wseq + 1) & 0xFF # Adding 1 to the d4 so loop repeates for d5 and so on
            time.sleep(0.15)

        # This code below basically tell the servo to save everything into its memory permanently
        shdr = [0x02, 0x42, 0x00, 0xf2, 0x03, 0x80, 0x00]
        sbody = [0x70, 0xff, 0xff, wseq & 0xFF]
        schk = sum(shdr[1:] + sbody) & 0xFF
        self.ser.write(bytes(shdr + sbody + [schk, 0x0a, 0x03]))
        wseq = (wseq + 1) & 0xFF
        time.sleep(0.3)


        # --- let the servo/DPC-CAN catch up from the burst of writes before
        #     trying to read anything back. ACTIVELY read and discard for a
        #     bit (don't sleep blind) - a blind sleep here stalls the USB
        #     pipe and makes the backlog worse, not better (we proved this
        #     with the read-side backlog issue earlier). ---
        settle_until = time.time() + 1.5
        while time.time() < settle_until:
            self.ser.read(256)
        self.ser.reset_input_buffer()


         # --- read each register back to confirm what actually stuck ---
        # IMPORTANT: use a FRESH sequence counter back in the proven-good
        # range here, rather than continuing to count up from the writes.
        # We found earlier (the ID1/ID2 bug) that brand-new/unused sequence
        # numbers get silently ignored, while numbers reused from the same
        # neighborhood as the wake-up sequence's own counter reliably work.
        # By the time the writes+save are done, wseq has drifted well past
        # that safe range - so we reset it here instead of reusing it.
        rseq = 0xd2
        confirmed = {}
        for register in values:
            value_back = self.read_register(register, rseq, timeout=1.5)
            rseq = (rseq + 1) & 0xFF
            value_back = HitecServo.to_signed16(value_back)
            confirmed[register] = value_back

        self.ser.close()
        return confirmed, None

    @staticmethod
    def to_signed16(v):
        """Converts raw 16-bit unsigned register values to signed values. 
        The servo transmits everything as a usigned register value because 
        some registers(like position), can be negative values"""
        if v is None:
            return None
        return v - 0x10000 if v >= 0x8000 else v
    

# ------------------------------------------
#  CubeMars Servo
# ------------------------------------------

CUBEMARS_TARGET_VID = 0x1A86
CUBEMARS_TARGET_PID = 0x7523
CUBEMARS_BAUD = 921600


class CubeMarsServo(Servo):

    # CAN baud labels for the GUI
    CUBEMARS_CAN_BAUD_LABELS = {0: "125K", 1: "250K", 2: "500K", 3: "1M",}

    CUBEMARS_MCCONF_OFFSETS = {
        "motor_max": 9,
        "motor_min": 13,
        "batt_max": 17,
        "batt_min": 21,
        "min_erpm": 29,
        "max_erpm": 33,
        "speed_kp": 324,
        "speed_ki": 328,
        "position_kp": 345,
        "position_ki": 349,
        "position_kd": 353,
    }

    CUBEMARS_APPCONF_OFFSETS = {
        "controler_id": 5,
        "timeout_ms": 6,
        "status_rate_hz": 16,
        "can_baud_rate": 17,
        "status_feedback_enable": None, # Not confirmed yet
        "brake_current_timeout": None, # Not confirmed yet 
    }

    # Display names for the cubeMars parameters on GUI
    CUBEMARS_DISPLAY_NAMES = {
        "motor_max": "Motor Max(Amps)",
        "motor_min": "Motor Min(Amps)",
        "batt_max": "Batt Max(Amps)",
        "batt_min": "Batt Min(Amps)",
        "min_erpm": "Min ERPM",
        "max_erpm": "Max ERPM",
        "speed_kp": "Speed KP",
        "speed_ki": "Speed KI",
        "position_kp": "Position KP",
        "position_ki": "Position KI",
        "position_kd": "Position KD",
    }
    
    def __init__(self):
        super().__init__(CUBEMARS_TARGET_VID, CUBEMARS_TARGET_PID, CUBEMARS_BAUD)
        
    @staticmethod
    def cubemars_crc16(data: bytes) -> int:
        """Computes the CRC16-XMODEM checksum of `data`, as required by the
        VESC packet protocol. This is a standard bit-by-bit CRC: for each
        byte, XOR it into the top of a 16-bit running value, then shift left
        8 times, XORing in the polynomial 0x1021 whenever the top bit is set.
        Returns the final 16-bit checksum."""
        crc = 0
        for byte in data:
            crc ^= (byte << 8)
            for _ in range(8):
                if crc & 0x8000:
                    crc = (crc << 1) ^ 0x1021
                else:
                    crc = crc << 1
                crc = crc & 0xFFFF
        return crc

    @staticmethod
    def cubemars_build_packet(payload: bytes) -> bytes:
        """Wraps a raw command `payload` in a full VESC packet: a start byte
        (0x02 for short payloads up to 255 bytes, 0x03 + a 2-byte length for
        longer ones), the payload itself, a 2-byte CRC16 (from
        cubemars_crc16), and a trailing stop byte (0x03). Returns the
        complete bytes object ready to write to the serial port."""
        crc = CubeMarsServo.cubemars_crc16(payload)
        crc_bytes = bytes([(crc >> 8) & 0xFF, crc & 0xFF])
        if len(payload) <= 255:
            return bytes([2, len(payload)]) + payload + crc_bytes + bytes([3])
        length = len(payload)
        length_bytes = bytes([(length >> 8) & 0xFF, length & 0xFF])
        return bytes([3]) + length_bytes + payload + crc_bytes + bytes([3])

    @staticmethod
    def cubemars_parse_response(raw: bytes):
        """Reverses cubemars_build_packet(): given raw bytes read back from
        the serial port, checks the start byte to figure out the header size
        and payload length, extracts the payload, and verifies the trailing
        CRC and stop byte. Returns the payload bytes if the frame is well
        formed and the CRC matches, otherwise None (covers truncated reads,
        corrupted bytes, or a malformed start byte)."""
        if len(raw) < 5:
            return None
        start = raw[0]
        if start == 2:
            length = raw[1]
            header_size = 2
        elif start == 3:
            length = (raw[1] << 8) | raw[2]
            header_size = 3
        else:
            return None
        payload = raw[header_size:header_size + length]
        crc_received = (raw[header_size + length] << 8) | raw[header_size + length + 1]
        stop = raw[header_size + length + 2]
        if stop != 3:
            return None
        if CubeMarsServo.cubemars_crc16(payload) != crc_received:
            return None
        return payload
    
    def cubemars_read_appconf(self):
        """Opens the port, reads app config, closes the port.
        Returns (payload_or_None, error_message_or_None) - same pattern as read_all_data."""
        port = self.find_port()
        if port is None:
            return None, "CH340/R-Link not found. Plug it in."

        self.ser = serial.Serial(port, self.baud, timeout=1)
        result = None
        for attempt in range(3):
            self.ser.write(CubeMarsServo.cubemars_build_packet(bytes([17])))
            time.sleep(0.3)
            response = self.ser.read(self.ser.in_waiting)
            result = CubeMarsServo.cubemars_parse_response(response)
            if result is not None and len(result) >= 18:
                break
            self.ser.reset_input_buffer()
            time.sleep(0.3)
        self.ser.close()

        if result is None or len(result) < 18:
            return None, "Failed to read CubeMars app config."
        return result, None

    @staticmethod
    def cubemars_get_appconf_field(appconf, field_name):
        offset = CubeMarsServo.CUBEMARS_APPCONF_OFFSETS[field_name]

        if field_name == "timeout_ms":
            return int.from_bytes(appconf[offset:offset+4], "big")
        else: 
            return appconf[offset]
            
    
    def cubemars_write_appconf(self, changes: dict):
        """Reads current config, edits only the given fields, writes it back,
        then re-reads to verify. Returns (verify_payload_or_None, error_message_or_None)."""

        current, error = self.cubemars_read_appconf()
        if error:
            return None, error

        data = bytearray(current)
        data[0] = 16 # COMM_SET_APPCONF

        
        for field_name, new_value in changes.items():
            offset = CubeMarsServo.CUBEMARS_APPCONF_OFFSETS[field_name]

            if self.CUBEMARS_APPCONF_OFFSETS[field_name] is not None:
                continue

            if field_name == "timeout_ms":
                data[offset:offset + 4] = new_value.to_bytes(4, "big")
            else: 
                data[offset] = new_value 

        port = self.find_port()
        if port is None:
            return None, "CH340/R-Link not found during write."

        self.ser = serial.Serial(port, self.baud, timeout=1)
        self.ser.write(CubeMarsServo.cubemars_build_packet(bytes(data)))
        time.sleep(0.5)
        self.ser.close()

        verify, error = self.cubemars_read_appconf()
        return verify, error
    
    def cubemars_read_mcconf(self):
        """Requests the full motor config (MCCONF) block from the controller
        via COMM_GET_MCCONF (command byte 14), retrying up to 3 times since
        the first reply after a cold connection sometimes gets dropped.
        Returns (payload_bytes, None) on success - the raw bytes, to be
        decoded field-by-field with cubemars_get_mcconf_field() - or
        (None, error_message) on failure."""
        port = self.find_port()
        if port is None:
            return None, "CH340/R-Link not found. Plug it in."
        
        self.ser = serial.Serial(port, self.baud, timeout=1)
        result = None
        for attempt in range(3):
            self.ser.write(CubeMarsServo.cubemars_build_packet(bytes([14])))
            time.sleep(0.3)
            response = self.ser.read(self.ser.in_waiting)
            result = CubeMarsServo.cubemars_parse_response(response)
            if result is not None and len(result) >= 357:
                break
            self.ser.reset_input_buffer()
            time.sleep(0.3)
        self.ser.close()

        if result is None or len(result) < 357:
            return None, "Failed to read CubeMars motor config."
        return result, None

    @staticmethod
    def cubemars_get_mcconf_field(mcconf, field_name):
        """Pulls one named field (e.g. 'speed_kp') out of a raw MCCONF byte
        blob returned by cubemars_read_mcconf(). Looks up the field's byte
        offset in CUBEMARS_MCCONF_OFFSETS, then unpacks 4 bytes there as a
        big-endian 32-bit float (struct format '>f'). Returns the float
        value."""
        offset = CubeMarsServo.CUBEMARS_MCCONF_OFFSETS[field_name]
        return struct.unpack(">f", mcconf[offset:offset + 4])[0]


    def cubemars_write_mcconf(self,changes: dict):
        """Read-modify-write for the motor config: reads the current MCCONF
        block, overwrites only the fields named in `changes` (a dict of
        field_name -> new float value, packed back to big-endian bytes at
        the right offset), sends the whole block back via COMM_SET_MCCONF
        (command byte 13), then re-reads to confirm. Returns whatever
        cubemars_read_mcconf() returns for that final confirmation read:
        (payload, None) or (None, error_message)."""
        current, error = self.cubemars_read_mcconf()
        if error:
            return None, error
        
        data=bytearray(current)
        data[0] = 13
        for field_name, new_value in changes.items():
            offset = CubeMarsServo.CUBEMARS_MCCONF_OFFSETS[field_name]
            data[offset:offset + 4] = struct.pack(">f", new_value)
        
        port = self.find_port()
        if port is None:
            return None, "CH340/R-Link not found during write."
        
        self.ser = serial.Serial(port, self.baud, timeout=1)
        self.ser.write(CubeMarsServo.cubemars_build_packet((bytes(data))))
        time.sleep(0.5)
        self.ser.close()

        return self.cubemars_read_mcconf()