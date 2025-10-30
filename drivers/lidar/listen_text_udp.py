# save as listen_text_udp.py
import socket
s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.bind(("0.0.0.0", 5005))
print("listening on UDP 5005 …")
while True:
    data, addr = s.recvfrom(4096)
    print(data.decode("utf-8", errors="replace").strip())
