import socket

# --- SETTINGS ---
LISTEN_IP = "0.0.0.0"  # Listen on all available network interfaces
LISTEN_PORT = 5001     # MUST match TELEMETRY_PORT in main.c
# ---

# Create a UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

# Bind the socket to the port
try:
    sock.bind((LISTEN_IP, LISTEN_PORT))
    print(f"--- Listening for telemetry on port {LISTEN_PORT} ---")
except OSError as e:
    print(f"Error: Could not bind to port {LISTEN_PORT}. Is another program using it?")
    print(e)
    exit()

# Loop forever, printing any data we receive
try:
    while True:
        data, addr = sock.recvfrom(1024)  # 1024 bytes buffer
        message = data.decode('utf-8')
        
        # We print an extra newline to separate the packets
        print(f"From {addr[0]}:\n{message}") 

except KeyboardInterrupt:
    print("\n--- Stopping listener ---")
    sock.close()