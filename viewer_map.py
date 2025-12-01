# viewer_map_debug.py — Updates on EVERY packet
import socket, struct, time, select
import numpy as np
import matplotlib.pyplot as plt

W, H   = 240, 240
PORT   = 5005
MAGIC  = 0x524D4150
HDR_FMT = "<I H B B"
HDR_SIZE = struct.calcsize(HDR_FMT)

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", PORT))
sock.setblocking(False)

print(f"--- DEBUG VIEWER LISTENING ON {PORT} ---")
print("This viewer draws ANY data it receives immediately.")

# Initialize buffer to 127 (Gray/Unknown)
buf = np.full((H, W), 127, dtype=np.uint8)
stripe_h, stripe_bytes, stripes_total = None, None, None

plt.ion()
fig = plt.figure(facecolor="black")
ax = plt.gca()
ax.set_facecolor("black")
plt.title("Debug Map (Partial Frames Allowed)", color="white")
plt.axis("off")

# Image placeholder
rgb = np.zeros((H, W, 3), dtype=np.uint8)
img = plt.imshow(rgb, origin="upper", interpolation="nearest")
plt.pause(0.001)

# Counters
packets_rx = 0

def map_to_rgb(src_u8, out_rgb):
    out_rgb[:] = 0  # Black background
    # Draw Unknown areas as Dark Gray
    m_unk = (src_u8 >= 120) & (src_u8 <= 135)
    out_rgb[m_unk] = (30, 30, 30)
    # Draw Walls as Bright Red
    m_occ = src_u8 >= 140
    out_rgb[..., 0][m_occ] = 255  # R
    out_rgb[..., 1][m_occ] = 0    # G
    out_rgb[..., 2][m_occ] = 0    # B

try:
    while plt.fignum_exists(fig.number):
        r, _, _ = select.select([sock], [], [], 0.01)
        if not r:
            plt.pause(0.001)
            continue

        try:
            data, _ = sock.recvfrom(65535)
        except BlockingIOError:
            continue

        # 1. Print Telemetry
        if len(data) < 100 and b"USS:" in data:
            print(f"[TELEM] {data.decode('utf-8').strip()}")
            continue

        # 2. Process Map Packet
        if len(data) < HDR_SIZE: continue
        magic, frame, stripe, total = struct.unpack_from(HDR_FMT, data, 0)
        if magic != MAGIC: continue

        # Init Geometry
        if stripes_total is None:
            stripes_total = int(total)
            stripe_h = H // stripes_total
            stripe_bytes = W * stripe_h
            print(f"[INIT] Stripes: {stripes_total}, Height: {stripe_h}")

        # Extract Payload
        payload = data[HDR_SIZE:]
        if len(payload) != stripe_bytes: continue

        # Update Buffer DIRECTLY (Don't wait for full frame)
        y0 = stripe * stripe_h
        stripe_arr = np.frombuffer(payload, dtype=np.uint8).reshape(stripe_h, W)
        buf[y0:y0+stripe_h, :] = stripe_arr
        
        packets_rx += 1
        
        # Update Display every 5 packets (to stay responsive)
        if packets_rx % 5 == 0:
            map_to_rgb(buf, rgb)
            img.set_data(rgb)
            plt.pause(0.001)

except KeyboardInterrupt:
    pass
finally:
    sock.close()