import serial
import json
import time
import sys
import time

baud_rate = 115220

def find_pico_port():
    """Try to find the Pico automatically"""
    import serial.tools.list_ports
    
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'Pico' in port.description or 'USB Serial' in port.description:
            return port.device
    return '/dev/ttyACM0'

    return '/dev/ttyACM0'

def get_float(message):
    while True:
        try:
            return float(input(message))
        except ValueError:
            continue

def get_int(message):
    while True:
        try:
            return int(input(message))
        except ValueError:
            continue


def print_menu():
    print("### Select the settings for the IMU reader system! ###")
    answer = input("use default values (y/n):\t")
    lpf_alpha = 0.95
    sample_rate = 120
    lpf_enabled = 1
    sensor_count = 3

    if answer != "y":
        sensor_count = get_int("Number of sensors(int)")
        lpf_enabled = input("LPF enabled? (y/n)")
        
        if (lpf_enabled.lower() == "y"):
            answer = get_float("LPF alpha [input 0 to use the default value!] (bigger more slow to react to changes)")
            if answer != 0:
                lpf_alpha = answer
        else:
            lpf_enabled = 0

        answer = abs(get_int("Sample rate?(int)[[input 0 to use the default value!]]"))
        if answer != 0:
            sample_rate = answer
    

    format_response = f"SC={sensor_count}|LPF_ENABLED={lpf_enabled}|LPF_ALPHA={lpf_alpha}|SR={sample_rate}|\n"

    return format_response


def receive_pico_data():
    port = find_pico_port() 
    counter = 0
    try:
        ser = serial.Serial(port, baud_rate)
        print(f"Connected to {port} at {baud_rate} baud")

        format_response = print_menu()
        ser.write(format_response.encode("utf-8"))
        print("Starting serial listening loop")
        while True:
            if ser.in_waiting > 0:
                # Read line from serial
                line = ser.readline().decode('utf-8').strip()
                if line:
                    print(f"{line}")
    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except KeyboardInterrupt:
        print("Stopping...")
    finally:
        if 'ser' in locals():
            ser.close()


if __name__ == "__main__":
    print(len(sys.argv))
    if len(sys.argv) > 1:
        baud_rate = 1200
    print(baud_rate)
    receive_pico_data()
