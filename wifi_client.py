import socket
import struct
import time
import pygame

# Define the data structure
class Data:
    def __init__(self, seq, left_x, left_y, right_x, right_y, l2_trigger, r2_trigger, r1_button, l1_button, x_button, y_button):
        self.seq = seq
        self.left_x = left_x
        self.left_y = left_y
        self.right_x = right_x
        self.right_y = right_y
        self.l2_trigger = l2_trigger
        self.r2_trigger = r2_trigger
        self.r1_button = r1_button
        self.l1_button = l1_button
        self.x_button = x_button
        self.y_button = y_button


# Function to delay for a specified number of milliseconds
def delay(milliseconds):
    time.sleep(milliseconds / 1000)


def apply_deadzone(value, deadzone=0.1):
    if -deadzone < value < deadzone:
        return 0
    else:
        return value


def main():
    # Attempt to connect to the server
    host = "192.168.50.193" #81  #193 # Server IP address
    port = 10000  # Server port
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((host, port))
        print("Connected to server successfully")
    except Exception as e:
        print("Error:", e)
        return
    
    pygame.init()
 
    # Initialize the joystick
    pygame.joystick.init()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    data = Data(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0)

    # Loop transmitting data to server every second
    done = False
    while not done:
        try:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    done = True
 
            # Read joystick values
            data.left_x = int(apply_deadzone(joystick.get_axis(0))*100)
            data.left_y = -int(apply_deadzone(joystick.get_axis(1))*100)
            data.right_x = int(apply_deadzone(joystick.get_axis(2))*100)
            data.right_y = -int(apply_deadzone(joystick.get_axis(3))*100)

            data.l2_trigger = int((apply_deadzone(joystick.get_axis(4)) + 1) * 50)
            data.r2_trigger = int((apply_deadzone(joystick.get_axis(5)) + 1) * 50)
    
            # Read button states
            data.r1_button = joystick.get_button(10)
            data.l1_button = joystick.get_button(9)
            data.x_button = joystick.get_button(0)
            data.y_button = joystick.get_button(1)
            print("Y:",data.y_button,"L1:",data.l1_button,"R1:", data.r1_button)
            # Send the data to the server
            packed_data = struct.pack("=7h4?", data.seq, data.left_x, data.left_y, data.right_x, data.right_y, data.l2_trigger, data.r2_trigger, data.r1_button, data.l1_button, data.x_button, data.y_button)
            client_socket.sendall(packed_data)
            # print(packed_data)
            # print(f"Data sent successfully: {len(packed_data)} bytes")

            data.seq += 1
            delay(65) # TODO: change this value to change the transfer speed

        except Exception as e:
            print("Error:", e)
            break

    client_socket.close()
    joystick.quit()


if __name__ == "__main__":
    main()

