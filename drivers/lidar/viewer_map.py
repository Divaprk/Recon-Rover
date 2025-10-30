# viewer_map.py — black bg, red walls, FPS overlay in the window
import socket, struct, time, select
import numpy as np
import matplotlib.pyplot as plt

W, H   = 240, 240
PORT   = 5005
MAGIC  = 0x524D4150                    # 'RMAP'
HDR_FMT = "<I H B B"                   # magic(uint32), frame(uint16), stripe(uint8), total(uint8)
HDR_SIZE = struct.calcsize(HDR_FMT)

# ---- socket: non-blocking so the UI never freezes ----
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("", PORT))
sock.setblocking(False)
print(f"listening on UDP {PORT} …")

# ---- state ----
buf = np.full((H, W), 127, dtype=np.uint8)   # 127 = unknown
last_frame = None
seen = set()

# stripe geometry (learned from the first packet)
stripe_h = None
stripe_bytes = None
stripes_total = None

# ---- matplotlib setup: black background, no axes ----
plt.ion()
fig = plt.figure(facecolor="black")
ax = plt.gca()
ax.set_facecolor("black")
title = plt.title("Occupancy Grid (live)", color="white")
plt.axis("off")

# FPS text (axes coords; y slightly above 1.0 puts it near the title)
fps_text = ax.text(
    0.99, 1.02, "0.0 fps",
    transform=ax.transAxes, ha="right", va="bottom",
    color="white", fontsize=11
)

# We’ll display an RGB image (H,W,3) derived from buf
rgb = np.zeros((H, W, 3), dtype=np.uint8)
img = plt.imshow(rgb, origin="upper", interpolation="nearest")
plt.pause(0.001)

def window_open() -> bool:
    return plt.fignum_exists(fig.number)

# ---- color mapping: black free, dark gray unknown, red occupied (intensity = confidence) ----
FREE_MAX     = 110
UNK_MIN      = 120
UNK_MAX      = 135
OCC_MIN      = 140
OCC_MAX      = 255

def map_to_rgb(src_u8: np.ndarray, out_rgb: np.ndarray) -> None:
    out_rgb[:] = 0  # default black
    m_unk = (src_u8 >= UNK_MIN) & (src_u8 <= UNK_MAX)
    out_rgb[m_unk] = (25, 25, 25)
    occ = np.clip(src_u8.astype(np.int16) - OCC_MIN, 0, OCC_MAX - OCC_MIN)
    red = (255.0 * (occ / float(OCC_MAX - OCC_MIN))**0.9).astype(np.uint8)
    m_occ = src_u8 >= OCC_MIN
    out_rgb[..., 0][m_occ] = red[m_occ]   # R

# FPS smoothing
fps_ema = 0.0
last_frame_time = time.time()

try:
    while window_open():
        # poll up to 50 ms so GUI stays responsive
        r, _, _ = select.select([sock], [], [], 0.05)
        if not r:
            map_to_rgb(buf, rgb)
            img.set_data(rgb)
            plt.pause(0.001)
            continue

        # read one full datagram (never blocks)
        try:
            data, _ = sock.recvfrom(65535)
        except BlockingIOError:
            continue

        if len(data) < HDR_SIZE:
            continue

        magic, frame, stripe, total = struct.unpack_from(HDR_FMT, data, 0)
        if magic != MAGIC:
            continue

        # first packet defines geometry
        if stripes_total is None:
            stripes_total = int(total)
            if stripes_total <= 0 or H % stripes_total != 0:
                print(f"bad total={total}; H={H} not divisible")
                break
            stripe_h = H // stripes_total
            stripe_bytes = W * stripe_h
            print(f"detected stripes_total={stripes_total}, stripe_h={stripe_h}, "
                  f"packet_payload≈{stripe_bytes + HDR_SIZE} bytes")

        if total != stripes_total:
            continue

        payload = data[HDR_SIZE:]
        if len(payload) != stripe_bytes:
            continue

        if last_frame != frame:
            last_frame = frame
            seen.clear()

        y0 = stripe * stripe_h
        stripe_arr = np.frombuffer(payload, dtype=np.uint8).reshape(stripe_h, W)
        buf[y0:y0+stripe_h, :] = stripe_arr
        seen.add(int(stripe))

        # update on first stripe and when the frame completes
        if stripe == 0 or len(seen) == stripes_total:
            map_to_rgb(buf, rgb)
            img.set_data(rgb)

            # update FPS when we complete a frame
            if len(seen) == stripes_total:
                now = time.time()
                dt = now - last_frame_time
                if dt > 0:
                    inst = 1.0 / dt
                    # Exponential moving average for stable display
                    fps_ema = inst if fps_ema == 0.0 else (0.9 * fps_ema + 0.1 * inst)
                    fps_text.set_text(f"{fps_ema:.2f} fps")
                last_frame_time = now

            plt.pause(0.001)

    print("viewer closed.")
except KeyboardInterrupt:
    pass
finally:
    sock.close()
