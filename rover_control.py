import socket
import keyboard
import time

ROVER_IP = "172.20.10.2"  # Replace with your rover's IP
ROVER_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print("WASD Rover Control")
print("W: Forward, A: Left, S: Backward, D: Right")
print("ESC: Exit")
print(f"Connecting to rover at {ROVER_IP}:{ROVER_PORT}")

try:
    while True:
        w = keyboard.is_pressed('w')
        a = keyboard.is_pressed('a')
        s = keyboard.is_pressed('s')
        d = keyboard.is_pressed('d')
        
        if keyboard.is_pressed('esc'):
            break
        elif w and a:
            sock.sendto(b"forward_left", (ROVER_IP, ROVER_PORT))
        elif w and d:
            sock.sendto(b"forward_right", (ROVER_IP, ROVER_PORT))
        elif s and a:
            sock.sendto(b"backward_left", (ROVER_IP, ROVER_PORT))
        elif s and d:
            sock.sendto(b"backward_right", (ROVER_IP, ROVER_PORT))
        elif w:
            sock.sendto(b"forward", (ROVER_IP, ROVER_PORT))
        elif s:
            sock.sendto(b"backward", (ROVER_IP, ROVER_PORT))
        elif a:
            sock.sendto(b"left", (ROVER_IP, ROVER_PORT))
        elif d:
            sock.sendto(b"right", (ROVER_IP, ROVER_PORT))
        else:
            sock.sendto(b"stop", (ROVER_IP, ROVER_PORT))
        
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    sock.sendto(b"stop", (ROVER_IP, ROVER_PORT))
    sock.close()
    print("\nRover control stopped")