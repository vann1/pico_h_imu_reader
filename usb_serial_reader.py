import serial
import json
import time
import sys
import time


BAUD_RATE = 115200

def find_pico_port():
    """Try to find the Pico automatically"""
    import serial.tools.list_ports
    
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if 'Pico' in port.description or 'USB Serial' in port.description:
            return port.device
    return '/dev/ttyACM0'

def receive_pico_data():
    port = find_pico_port() 
    counter = 0

    try:
        ser = serial.Serial(port, BAUD_RATE, timeout=1)
        print(f"Connected to {port} at {BAUD_RATE} baud")
        start_time = time.time()

        while True:
            if ser.in_waiting > 0:
                # Read line from serial
                line = ser.readline().decode('utf-8').strip()
                if line:
                    elapsed_time = time.time() - start_time
                    counter += 1
                    if elapsed_time >=1:
                        print(f"{counter}")
                        counter = 0
                        start_time = time.time()
                    print(f"Raw: {line}")
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
        BAUD_RATE = 1200
    print(BAUD_RATE)
    receive_pico_data()
