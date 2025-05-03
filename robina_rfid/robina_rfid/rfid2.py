import serial
import time

SERIAL_PORT = "/dev/ttyUSB0"  # Change as needed
BAUD_RATE = 115200

START_TAG_READING_CMD = bytes.fromhex("43 4D 02 02 02 00 00 00 00")
STOP_TAG_READING_CMD  = bytes.fromhex("43 4D 03 02 02 00 00 00 00")

# Enable only Antenna 1 (with power=30dBm, on=200ms, off=100ms)
SET_ANTENNA_1_CMD = bytes.fromhex(
    "43 4D 86 02 1D 00 00 00"
    "00"        # save config
    "04"        # number of antennas
    "01"        # enable bitmask (only antenna 1 on)
    "01 1E 00 C8 00 64"  # Ant 1
    "02 1E 00 C8 00 64"  # Ant 2
    "03 1E 00 C8 00 64"  # Ant 3
    "04 1E 00 C8 00 64"  # Ant 4
    "09"        # checksum
)

def parse_tags(raw_bytes):
    tags = []
    i = 0
    while i < len(raw_bytes) - 10:
        if raw_bytes[i:i+2] != b'\x43\x4D':
            i += 1
            continue

        try:
            length = raw_bytes[i+4] + (raw_bytes[i+5] << 8)
            frame_end = i + 8 + length + 1
            frame = raw_bytes[i:frame_end]

            if len(frame) < 20:
                i += 1
                continue

            tag_class = frame[8]
            tag_len = frame[9]
            tag_id_start = 11
            tag_id_end = tag_id_start + tag_len
            tag_id = frame[tag_id_start:tag_id_end].hex().upper()

            tags.append(tag_id)
            i = frame_end
        except Exception as e:
            i += 1
            continue

    return tags

def main():
    try:
        ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
        print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

        ser.write(SET_ANTENNA_1_CMD)
        print("Set to use antenna 1 only.")
        time.sleep(0.2)

        ser.write(START_TAG_READING_CMD)
        print("Started tag reading for 10 seconds...")

        start_time = time.time()
        buffer = bytearray()
        all_tags = set()  # Use a set to avoid duplicate tags

        while time.time() - start_time < 10:
            if ser.in_waiting:
                buffer += ser.read(ser.in_waiting)

            tags = parse_tags(buffer)
            all_tags.update(tags)  # Add detected tags to the set

            buffer.clear()  # Clear after parsing
            time.sleep(0.1)

        ser.write(STOP_TAG_READING_CMD)
        print("Stopped tag reading.")
        ser.close()
        print("Serial port closed.")

        # Print all detected tags at once
        print("Detected Tags:")
        for tag in all_tags:
            print(tag)

    except serial.SerialException as e:
        print("Serial error:", e)
    except KeyboardInterrupt:
        print("\nInterrupted.")
        ser.write(STOP_TAG_READING_CMD)
        ser.close()

if __name__ == "__main__":
    main()
