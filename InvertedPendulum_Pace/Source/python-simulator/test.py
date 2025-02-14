# writer
import serial
import time
import json
import random

# writer_port_name = '/dev/pts/12'  # Replace with the appropriate serial port name
listener_port_name = '/dev/pts/7' #'/dev/ttyACM0'

try:
    # Open the serial port
    listener_ser = serial.Serial(listener_port_name)
    print(f"Serial port '{listener_ser.name}' opened successfully.")
    listener_ser.flush()

    while True:
        line = listener_ser.readline().decode().strip("'")
        # received_data = json.loads(line)
        print(line)

except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    # Close the serial port
    if listener_ser and listener_ser.is_open:
        listener_ser.close()
        print("Serial port closed.")
