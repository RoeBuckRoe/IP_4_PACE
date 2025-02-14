"""This module read from and write to a virtual serial port."""
import serial
import random
import struct
import time

serial_port_name: str ='/dev/ttyACM1' # '/dev/pts/8'  # Replace with the appropriate serial port name

OFFSET_LOWER_RANGE: float = -0.001
OFFSET_UPPER_RANGE: float = 0.001

def oppositeSigns(x: int, y: int) -> bool:
    return x**y < 0

try:
    # Opens the serial port
    serial_port: serial.Serial = serial.Serial(serial_port_name, 230400, timeout=5)
    serial_port.reset_input_buffer()
    serial_port.reset_output_buffer()
    print(f"Serial port '{serial_port.name}' opened successfully.")

    encoder_position: int = 0
    previous_encoder_position: int = 0
    motor_position: int = 0
    encoder_position_init: int = 0
    peaked: bool = False
    global_max_encoder_position: int = 0
    max_encoder_position: int = 0


    while True:

        
        ba = bytearray(struct.pack("iIB", 4, 4, 4 ))
        serial_port.write( ba )

        data_received = serial_port.read(25)
        print(" Received : ", len(data_received) )
        if len(data_received) == 25:
            unpacked_data = struct.unpack("iiIIiIB", data_received)
            print(unpacked_data)

        #time.sleep(15)
        continue

        ba = bytearray(struct.pack("4i", 1, 1, 1, 1 ))
        ba = bytearray(struct.pack("4i", random.randint(0, 32768), random.randint(0, 32768), random.randint(0, 32768), random.randint(0, 32768)))
        serial_port.write( ba )
        continue
        #print('listening .... ')
        #print( serial_port.readline() )
        
        

        received_messages: list = serial_port.readline().decode().split(";")

        # Sedning the 16 bytes
        # 

        

        # Splitting received messages into separate commands, separated by ;
        for message in received_messages[:-1]:
            # Splitting each message into command and parameters
            try:
                command_index: int = int(message[0])
                parameters: list = message.split(",")[1:]
                if command_index == 0: # Initialisation
                    encoder_position, motor_position = 0, 0
                    message: str = f"1,0,Initialisation_successful\n"
                    serial_port.write(message.encode())
                    # print(f"cmd_idx = {message[0]} successful!")

                elif command_index == 1: # Read current encoder position and motor position
                    # encoder position part
                    cnt3: int = random.randint(0, 32768)
                    
                    if (cnt3 >= 32768):
                        encoder_position = int(cnt3)
                        encoder_position -= 65536
                    else:
                        encoder_position = int(cnt3)

                    range_error = 0
                    if encoder_position <= -32768:
                        range_error = -1
                        encoder_position = -32768
                    if encoder_position >= 32767:
                        range_error = 1
                        encoder_position = 32767

                    encoder_position = encoder_position - encoder_position_init

                    #   Detect if we passed the bottom, then re-arm peak flag
                    #   oppositeSigns returns true when we pass the bottom position

                    if (oppositeSigns(encoder_position, previous_encoder_position)):
                        peaked = 0
                        zero_crossed: int = 1

                    if not peaked: # We don't need to evaluate anymore if we hit a maximum when we're still in downward motion and didn't cross the minimum
                        # Add global maximum
                        if (abs(encoder_position) >= abs(global_max_encoder_position)):
                            global_max_encoder_position = encoder_position
                        # Check if new maximum
                        if (abs(encoder_position) >= abs(max_encoder_position)):
                            max_encoder_position = encoder_position
                        else:
                        # We are at the peak and disable further checks until we traversed the minimum position again
                            peaked = 1
                            handled_peak: int = 0

                    previous_encoder_position = encoder_position

                    # motor position part
                    offset: float = random.uniform(OFFSET_LOWER_RANGE, OFFSET_UPPER_RANGE)
                    temp_motor_position: float = motor_position + offset
                    message: str = f"0,{float(previous_encoder_position)},{temp_motor_position}\n"
                    serial_port.write(message.encode())

                elif command_index == 2: # Set motor position
                    motor_position = float(parameters[0])
                    message: str = f"1,0,Motor_position_set_successful\n"
                    serial_port.write(message.encode())
                
                else:
                    raise IndexError("command index not found")

            except (ValueError, OverflowError, IndexError) as e:
                # error_type = type(e).__name__
                # print(error_type)
                message: str = f"1,1,{e}\n"
                serial_port.write(message.encode())

except (serial.SerialException) as e:
    print(f"Error: {e}")
finally:
    # Closes the serial port
    if serial_port.is_open:
        serial_port.close()
        print("Serial port closed.")

