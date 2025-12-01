import socket
import keyboard
import time

# --- CONFIG ---
ROVER_IP = "172.20.10.2"  # <--- Make sure this matches your Pico IP!
ROVER_PORT = 5005 
# --------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

print(f"Connecting to {ROVER_IP}:{ROVER_PORT}")
print("WASD: Drive | L: Toggle LIDAR | ESC: Exit")

try:
    while True:
        if keyboard.is_pressed('esc'): break

        # --- TOGGLE LIDAR ---
        if keyboard.is_pressed('l'):
            print("Sending: Toggle LIDAR")
            sock.sendto(b"toggle_lidar", (ROVER_IP, ROVER_PORT))
            time.sleep(0.5) # Debounce so it doesn't toggle 10 times

        # --- DRIVE COMMANDS ---
        if keyboard.is_pressed('w') and keyboard.is_pressed('a'):
            sock.sendto(b"forward_left", (ROVER_IP, ROVER_PORT))
        elif keyboard.is_pressed('w') and keyboard.is_pressed('d'):
            sock.sendto(b"forward_right", (ROVER_IP, ROVER_PORT))
        elif keyboard.is_pressed('s') and keyboard.is_pressed('a'):
            sock.sendto(b"backward_left", (ROVER_IP, ROVER_PORT))
        elif keyboard.is_pressed('s') and keyboard.is_pressed('d'):
            sock.sendto(b"backward_right", (ROVER_IP, ROVER_PORT))
        elif keyboard.is_pressed('w'):
            sock.sendto(b"forward", (ROVER_IP, ROVER_PORT))
        elif keyboard.is_pressed('s'):
            sock.sendto(b"backward", (ROVER_IP, ROVER_PORT))
        elif keyboard.is_pressed('a'):
            sock.sendto(b"left", (ROVER_IP, ROVER_PORT))
        elif keyboard.is_pressed('d'):
            sock.sendto(b"right", (ROVER_IP, ROVER_PORT))
        else:
            sock.sendto(b"stop", (ROVER_IP, ROVER_PORT))
        
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    sock.close()