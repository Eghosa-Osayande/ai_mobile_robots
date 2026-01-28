# MSC ROBOTICS AND AUTOMATION. SCHOOL OF SCIENCE, ENGINEERING AND ENVIRONMENT. UNIVERSITY OF SALFORD
# Terminal-based Pioneer P3-DX control (no GUI, keyboard control)

import time
import threading
import struct
import time
import serial


COM_PORT = "/dev/cu.usbserial-10"
BAUD_RATE = 9600

# Synchronization packets
HEADER = bytes([0xFA, 0xFB])
SYNC0 = bytes([250, 251, 3, 0, 0, 0])
SYNC1 = bytes([250, 251, 3, 1, 0, 1])
SYNC2 = bytes([250, 251, 3, 2, 0, 2])

def try_serial_connection():
    try:
        ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
        return ser
    except Exception as e:
        print("Serial connection failed. Attempting TCP connection...")
        print(e)
        return None


def calc_checksum(packet):
    c = 0
    i = 3
    n = packet[2] - 2
    while n > 1:
        c += (packet[i] << 8) | packet[i + 1]
        c &= 0xFFFF
        n -= 2
        i += 2
    if n > 0:
        c ^= packet[i]
    return c


connection = try_serial_connection()

def send_command(command_number, argument_data=1, argument_type="byte"):
    COMMAND_NUMBER = bytes([command_number])
    if argument_type == "byte":
        if argument_data < 0:
            ARGUMENT_TYPE = bytes([0x1B])
            argument_data = argument_data * -1
        else:
            ARGUMENT_TYPE = bytes([0x3B])
    else:
        ARGUMENT_TYPE = bytes([0x2B])  # String argument
    ARGUMENT_DATA = struct.pack(">H", argument_data)
    BYTE_COUNT = len(COMMAND_NUMBER) + len(ARGUMENT_TYPE) + len(ARGUMENT_DATA) + 2
    checksum_data = (
        HEADER + bytes([BYTE_COUNT]) + COMMAND_NUMBER + ARGUMENT_TYPE + ARGUMENT_DATA
    )
    checksum = calc_checksum(checksum_data).to_bytes(2, "big")
    packet = (
        HEADER
        + bytes([BYTE_COUNT])
        + COMMAND_NUMBER
        + ARGUMENT_TYPE
        + ARGUMENT_DATA
        + checksum
    )

    connection.write(packet)


# -------------------- SIP PARSING --------------------


def parse_main_sip(sip_response):
    if sip_response[:2] != bytes([0xFA, 0xFB]):
        return None

    data = {
        "byte_count": sip_response[2],
        "xpos": sip_response[3],
        "ypos": sip_response[4],
        "theta": sip_response[5],
        "lvel": sip_response[6],
        "rvel": sip_response[7],
        "battery": sip_response[8],
        "stall_bumpers": sip_response[9],
        "control": sip_response[10],
        "flags": sip_response[11],
        "compass": sip_response[12],
        "sonar_count": sip_response[13],
        "sonar_number": sip_response[14],
    }

    sonars = []
    for i in range(16):
        idx = 24 + i * 3
        sonars.append(struct.unpack("<h", sip_response[idx : idx + 2])[0])
    data["sonars"] = sonars
    return data


def receive_sips():
    sip_response = connection.read(1024)
    return parse_main_sip(sip_response)


def get_multiple_values(keys):
    data = receive_sips()
    if data:
        return {key: data.get(key) for key in keys}
    return {}


def getX():
    return receive_sips().get("xpos", None)


def getY():
    return receive_sips().get("ypos", None)


def getTh():
    return receive_sips().get("theta", None)


def getLeftVel():
    return receive_sips().get("lvel", None)


def getRightVel():
    return receive_sips().get("rvel", None)


def getSonarRange(index):
    data = receive_sips()
    if data:
        sonars = data.get("sonars", [])
        if 0 <= index < len(sonars):
            return sonars[index]
    return None


def enableMotors(enable):
    send_command(4, argument_data=1 if enable else 0)


def setAccel(accel):
    send_command(5, argument_data=accel)


def move(distance):
    send_command(8, argument_data=distance)


def rotate(degrees):
    direction = 1 if degrees > 0 else -1
    send_command(21, argument_data=direction)
    time.sleep(0.5)
    send_command(29)


def setRV(velocity):
    send_command(10, argument_data=velocity)


def rVel():
    send_command(21)


def stop():
    send_command(29)

def send_initial_packets():
    sync_packets = [SYNC0, SYNC1, SYNC2]
    for packet in sync_packets:
        connection.write(packet)
        response = connection.read(1024)

        if response != packet and packet != SYNC2:
            print(f"Error: {packet} response mismatch")
            return False
        elif packet == SYNC2:
            header = response[:2]
            if header != bytes([0xFA, 0xFB]):
                print("Error: Invalid SIP header")
                return
            print(response[3:29].decode("utf-8", errors="ignore"))

    print("Connection established successfully")
    send_command(1)  # open
    send_command(4, argument_data=1)  # enable motors
    return True


def watchdog_pulse(stop_event, interval=1.5):
    while not stop_event.is_set():
        send_command(0)  # PULSE
        time.sleep(interval)


stop_event = threading.Event()
pulse_thread = threading.Thread(target=watchdog_pulse, args=(stop_event,), daemon=True)


def main():
    duration = 0.15

    try:
        if send_initial_packets():
            print("Terminal control active")
            print("1 = forward | 2 = backward | 3 = left | 4 = right | 0 = exit")
            pulse_thread.start()
            while True:
                x = input("Enter ")
                print(x)

                if x == "1":
                    move(1)
                    time.sleep(duration)

                elif x == "2":
                    move(-1)
                    time.sleep(duration)

                elif x == "3":
                    rotate(1)
                    time.sleep(duration)

                elif x == "4":
                    rotate(-1)
                    time.sleep(duration)

                elif x == "0":
                    print("Stopping robot and exiting")
                    stop()
                    break

                # Print robot state
                data = get_multiple_values(
                    ["xpos", "ypos", "theta", "lvel", "rvel", "sonars"]
                )
                print(data)

    except Exception as e:
        print("Error:", e)

    except KeyboardInterrupt as e:
        print("User cancelled")

    finally:
        stop()
        if connection:
            connection.close()
        print("Connection closed")


if __name__ == "__main__":
    main()
    pass
