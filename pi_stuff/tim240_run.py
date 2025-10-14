#!/usr/bin/env python3
import socket, time

# --- import your logic as-is ---
from tim240_logic import HammerRearDetector

# ---- EASY HAND-TEST PRESETS (feel free to tune) ----
# These override the detector thresholds after we create it
TEST_THRESHOLDS = dict(
    # Treat anything closer than ~0.30 m as "hammer/occlusion"
    D_CLOSE=0.30,
    # Treat a "person/hand" anywhere between 0.30 m and 1.20 m
    D_PERSON_MIN=0.30,
    D_PERSON_MAX=1.20,
    # Dwell short so your hand movement is responsive
    DWELL_MS=100,
    MIN_POINTS_IN_ROI=1,   # was 6; we only provide ONE point now
)

# ---- LiDAR connection ----
HOST, PORT = "192.168.0.20", 2111
STX, ETX   = b"\x02", b"\x03"

def send(sock, cmd, expect_reply=True, timeout=2.0):
    sock.settimeout(timeout)
    sock.sendall(STX + cmd.encode("ascii") + ETX)
    if not expect_reply:
        return b""
    try:
        return sock.recv(65535)
    except socket.timeout:
        return b""

# ---- Parser for sSN LMDscandata (ASCII/hex) ----
def parse_lmd(frame_text):
    """
    Extract only the 0° beam from sSN LMDscandata.
    Returns [(0.0, distance_m)] or None if frame isn't LMDscandata.
    """
    line = " ".join(frame_text.strip().split())
    if not line.startswith("sSN LMDscandata"):
        return None

    toks = line.split()
    try:
        i = toks.index("DIST1")
    except ValueError:
        return None

    # helper: 32-bit signed hex -> int
    def h2s32(h):
        v = int(h, 16)
        return v - (1 << 32) if v & (1 << 31) else v

    try:
        scale        = int(toks[i+1], 16)        # usually 0x3F800000 (1.0f)
        scale_offset = int(toks[i+2], 16)        # usually 0
        start_deg    = h2s32(toks[i+3]) / 10000.0  # signed, 1/10000°
        step_deg     = int(toks[i+4], 16) / 10000.0
        count        = int(toks[i+5], 16)
    except Exception:
        return None

    j = i + 6
    try:
        k = toks.index("RSSI1", j)
    except ValueError:
        k = len(toks)
    dist_hex = toks[j : min(k, j + count)]
    if not dist_hex:
        return None

    # index for 0° (clamped into range)
    if step_deg <= 0:
        return None
    idx0 = round((0.0 - start_deg) / step_deg)
    idx0 = max(0, min(idx0, len(dist_hex) - 1))

    # convert that one sample to metres (mm -> m)
    try:
        d_m = int(dist_hex[idx0], 16) / 1000.0
    except ValueError:
        d_m = 0.0

    return [(0.0, d_m)]



def main():
    # Build detector, then override a few thresholds for hand testing
    det = HammerRearDetector()
    for name, val in TEST_THRESHOLDS.items():
        # overwrite the global constants on the class' module if present
        if hasattr(det, name):
            setattr(det, name, val)  # in case you moved constants into the instance
    # Also patch module constants your detector references (common pattern)
    import tim240_logic as L
    for name, val in TEST_THRESHOLDS.items():
        if hasattr(L, name):
            setattr(L, name, val)

    # Connect
    print(f"Connecting to TiM240 at {HOST}:{PORT}")
    s = socket.create_connection((HOST, PORT), timeout=3)

    # Sanity check
    ident = send(s, "sRN DeviceIdent").decode(errors="ignore").strip()
    print("Ident:", ident if ident else "(no reply)")

    # Enable stream
    print("Enabling stream...")
    send(s, "sEN LMDscandata 1", expect_reply=False)

    # stream loop
    buf = b""
    last_state = None
    s.settimeout(1.0)

    try:
        while True:
            try:
                chunk = s.recv(65535)
                if not chunk:
                    continue
                buf += chunk

                # split by ETX; frames are STX...ETX, but STX isn't always at buffer start.
                while ETX in buf:
                    frame, buf = buf.split(ETX, 1)
                    # keep only text after the last STX in the frame part
                    if STX in frame:
                        frame = frame.split(STX)[-1]
                    text = frame.decode("ascii", errors="ignore")
                    pts = parse_lmd(text)
                    if pts is None:
                        continue

                    # get time first (so it's defined everywhere below)
                    now_ms = int(time.time() * 1000)

                    # Optional: show the raw 0° reading every 2 s for sanity
                    if pts:
                        raw0 = pts[0][1]
                        if now_ms - getattr(det, "_last_raw_print", 0) >= 2000:
                            print(f"{time.strftime('%H:%M:%S')}  raw_0deg={raw0:.3f} m")
                            det._last_raw_print = now_ms
                    out = det.update(pts, now_ms, sim_mode="unknown")
                    wd = det.watchdog(now_ms)
                    if wd:
                        out = wd
                    
                    # --- only print every 2 seconds ---
                    
                    if now_ms - getattr(det, "_last_print", 0) >= 2000:
                        dmin = out.get("dmin")
                        state = out.get("state")
                        # handle missing dmin gracefully
                        if dmin is None:
                            print(f"{time.strftime('%H:%M:%S')}  state={state}  dmin=None")
                        else:
                            print(f"{time.strftime('%H:%M:%S')}  state={state}  dmin={dmin:.2f} m")
                        det._last_print = now_ms



                    # Print only on change or every ~0.5 s
                    state = out.get("state")
                    dmin  = out.get("dmin")
                    if state != last_state:
                        print(f"{time.strftime('%H:%M:%S')}  state={state}  dmin={dmin} m")
                        last_state = state

            except socket.timeout:
                # no data this tick; watchdog will handle prolonged gaps
                now_ms = int(time.time() * 1000)
                wd = det.watchdog(now_ms)
                if wd and wd.get("state") != last_state:
                    print(f"{time.strftime('%H:%M:%S')}  state={wd['state']} ({wd.get('reason')})")
                    last_state = wd.get("state")

    except KeyboardInterrupt:
        pass
    finally:
        print("\nDisabling stream and closing...")
        try:
            send(s, "sEN LMDscandata 0", expect_reply=False)
        except Exception:
            pass
        s.close()

if __name__ == "__main__":
    main()
