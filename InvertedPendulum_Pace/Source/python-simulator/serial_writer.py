import serial
import time
import json
import random

writer_port_name = '/dev/pts/8'  # Replace with the appropriate serial port name
listener_port_name = '/dev/pts/7'

try:
    # Open the serial port
    writer_ser = serial.Serial(writer_port_name, 230400)
    print(f"Serial port '{writer_ser.name}' opened successfully.")

    listener_ser = serial.Serial(listener_port_name)
    print(f"Serial port '{listener_ser.name}' opened successfully.")
    listener_ser.flush()

    while True:
        # x = ser.read()        # read one byte
        # if listener_ser.readline():
        line = listener_ser.readline().decode().strip("'")
        received_data = json.loads(line)
        try:
            encoder_position = float(received_data["encoder_position"])
            rotor_position = float(received_data["rotor_position"])
            is_stable = received_data["is_stable"]
            print("enc pos", encoder_position, "rot pos", rotor_position)
            if not is_stable:
                # Message to be sent
                motor_control = 6
                test_dict = {'motor_control': motor_control}
                test_data = json.dumps(test_dict).encode()
                writer_ser.write(test_data)
                writer_ser.write(b'\n')
        except ValueError:
            pass

except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    # Close the serial port
    if writer_ser.is_open:
        writer_ser.close()
        print("Serial port closed.")



