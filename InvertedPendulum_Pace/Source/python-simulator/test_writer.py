# writer
import serial
import time
import json
import random
import struct

writer_port_name = '/dev/pts/8'  # Replace with the appropriate serial port name

try:
    # Open the serial port
    writer_ser = serial.Serial(writer_port_name)
    print(f"Serial port '{writer_ser.name}' opened successfully.")

    while True:
        # motor_control = random.randint(20, 30)
        # motor_control = random.randint(20, 30)
        # test_dict = {'motor_control': motor_control}
        # test_data = json.dumps(test_dict).encode()
        # text separated by , and ;
        # TODO: Generate a string with , and ;
        # TODO: Don't need to use json, processing as well
        # writer_ser.write(test_data)

        #writer_ser.write(b'0;2,2;1;0;2,2;\n')
        # writer_ser.write(b'\n')

        ba = bytearray(struct.pack("fiii", 345.678, 7, 20, 1, 200,5,4))
        writer_ser.write( ba )
        time.sleep(15)
        continue


        time.sleep(10)

except serial.SerialException as e:
    print(f"Error: {e}")
finally:
    # Close the serial port
    if writer_ser.is_open:
        writer_ser.close()
        print("Serial port closed.")



# command_index = (0, 1, 2, 3) # what to do
# response = (0, 1) # me telling you
# 1: 1,0(all ok)/1(some error message),status(str)
# 0: 0, encoder position, rotor position

