import serial.tools.list_ports


def main():
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("No serial ports found.")
        return
    print(f"{len(ports)} serial port(s) found:\n")
    for p in ports:
        print(f"  device      : {p.device}")
        print(f"  description : {p.description}")
        print(f"  hwid        : {p.hwid}")
        print(f"  manufacturer: {p.manufacturer}")
        print(f"  vid:pid     : "
              f"{f'{p.vid:04x}:{p.pid:04x}' if p.vid is not None else 'n/a'}")
        print()


if __name__ == "__main__":
    main()