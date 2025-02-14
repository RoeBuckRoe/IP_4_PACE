# writer
import serial
import time
import json
import random

# writer_port_name = '/dev/pts/12'  # Replace with the appropriate serial port name
listener_port_name = '/dev/pts/7'

try:
    # Open the serial port
    listener_ser = serial.Serial(listener_port_name, 230400)
    print(f"Serial port '{listener_ser.name}' opened successfully.")
    listener_ser.flush()

    while True:
        line = listener_ser.readline().decode()
        # received_data = json.loads(line)
        print(line)

    # Publish interval (seconds)
#     interval = 0.1
#     time_rotated = 1
#     while rotor_position < 3.14:
#         print("YES")
#         # x = ser.read()        # read one byte
#         line = listener_ser.readline().decode().strip("'")
#         received_data = json.loads(line)
#         try:
#             # float(line)
#             motor_control = float(received_data["motor_control"])
#             print("mot_ctrl", motor_control)
#             motor_position -= movement_from_2_degrees * amplifier * motor_control
#             motor_position = abs(motor_position)
#             rotor_position = ((0.803 ** 2) * (time_rotated ** 2)) / ((time_rotated ** 2) + (55.85 ** 0.5)*time_rotated/20 + 55.85) * motor_position * 2
#             test_dict = {'encoder_position': round(motor_position, 8), 'rotor_position': round(rotor_position, 8), 'is_stable': False}
#             test_data = json.dumps(test_dict).encode()
#             writer_ser.write(test_data)
#             writer_ser.write(b'\n')
#             amplifier += 2
#             time_rotated += 1
#             # print("Message sent:", message)

#             # Wait for the specified interval before sending the next message
#             time.sleep(interval)

#         except ValueError:
#             pass

#     curr_time = 0
#     rotate_left = True
#     while True:
#         random_time = random.randint(1, 10)
#         while curr_time < random_time:
#             if rotate_left:
#                 motor_position += 0.1
#             else:
#                 motor_position -= 0.1
#             rotor_position = 3.14 + random.uniform(-0.1, 0.1)
#             test_dict = {'encoder_position': round(motor_position, 8), 'rotor_position': round(rotor_position, 8), 'is_stable': True}
#             test_data = json.dumps(test_dict).encode()
#             writer_ser.write(test_data)
#             writer_ser.write(b'\n')
#             curr_time += 1
#             time.sleep(interval) 
#         curr_time = 0
#         rotate_left = not rotate_left
    



except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    # Close the serial port
    if listener_ser.is_open:
        listener_ser.close()
        print("Serial port closed.")
