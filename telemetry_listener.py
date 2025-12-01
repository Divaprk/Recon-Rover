import socket

# --- UPDATE THIS ---
LISTEN_IP = "0.0.0.0"
LISTEN_PORT = 5005     # <--- MUST BE 5005
# -------------------

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((LISTEN_IP, LISTEN_PORT))

print(f"Listening for data on port {LISTEN_PORT}...")

try:
    while True:
        data, addr = sock.recvfrom(65535)
        # Try to decode as text (telemetry)
        try:
            msg = data.decode('utf-8')
            # If it starts with "USS", it's telemetry. If not, it might be map binary data.
            if msg.startswith("USS"):
                print(msg.strip())
        except:
            # If decode fails, it was probably binary map data
            pass
except KeyboardInterrupt:
    sock.close()