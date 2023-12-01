import pygame
import socket
import struct
import time
 
pygame.init()
 
# Initialize the joystick
pygame.joystick.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()
 
# Server address and port
server_address = '192.168.50.93'  # Replace with the IP address of your ESP32
server_port = 10000
 
# Create a socket
client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
def apply_deadzone(value, deadzone=0.1):
    if -deadzone < value < deadzone:
        return 0
    else:
        return value
try:
    # Connect to the server
    client_socket.connect((server_address, server_port))
 
    done = False
    while not done:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                done = True
 
        # Read and print the left Y-axis, right X-axis, and right Y-axis values
        left_y = apply_deadzone(joystick.get_axis(1))
        right_x = apply_deadzone(joystick.get_axis(3))
        l2_trigger = apply_deadzone(joystick.get_axis(2))
        r2_trigger = apply_deadzone(joystick.get_axis(5))
 
        # Read button states
        r1_button = joystick.get_button(5)
        l1_button = joystick.get_button(4)
        x_button = joystick.get_button(0)
        print(int(-left_y*100),int(right_x*100),int((l2_trigger + 1) * 50),int((r2_trigger + 1) * 50),r1_button,l1_button,x_button)
        print(b'IY')
        print(struct.pack('i', int(-left_y*100)))
        client_socket.send(b'IY')  # Send type identifier for integer
        client_socket.send(struct.pack('i', int(-left_y*100)))
        
        client_socket.send(b'IX')
        client_socket.send(struct.pack('i', int(right_x*100)))
 
        client_socket.send(b'IS')
        client_socket.send(struct.pack('i', int(x_button)))
        
        client_socket.send(b'IF')
        client_socket.send(struct.pack('i', int((l2_trigger + 1) * 50)))
        
        client_socket.send(b'IB')
        client_socket.send(struct.pack('i', int((r2_trigger + 1) * 50)))
 
        client_socket.send(b'IC')
        client_socket.send(struct.pack('i', int(r1_button)))
 
        client_socket.send(b'IR')
        client_socket.send(struct.pack('i', int(l1_button)))
        
        # # Add a short delay to control the sending rate
        time.sleep(0.1)  # Adjust the delay as needed
 
finally:
    # Close the socket and the joystick (usually not reached in an infinite loop)
    client_socket.close()
    joystick.quit()