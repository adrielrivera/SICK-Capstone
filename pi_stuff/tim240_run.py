#!/usr/bin/env python3
import socket, time

# --- Kite-shaped field detection system ---
# No longer using the old single-point detector

# ---- Kite-shaped field detection function ----
def analyze_kite_field(points):
    """
    Analyze points within the kite-shaped field for safety violations.
    Returns dict with detection results.
    """
    if not points:
        return {"state": "FAILSAFE", "reason": "no_points", "violations": []}
    
    violations = []
    hammer_detected = False
    person_detected = False
    
    for angle_deg, distance_m in points:
        # Calculate safe distance for this angle
        safe_distance_m = calculate_kite_safe_distance(angle_deg)
        
        # Check for hammer (very close objects)
        if distance_m < (HAMMER_MAX_DISTANCE / 100.0):  # Convert cm to m
            hammer_detected = True
            violations.append({
                "type": "hammer",
                "angle": angle_deg,
                "distance": distance_m,
                "safe_distance": safe_distance_m
            })
        
        # Check for person (between hammer distance and safe distance)
        elif distance_m < safe_distance_m:
            person_detected = True
            violations.append({
                "type": "person",
                "angle": angle_deg,
                "distance": distance_m,
                "safe_distance": safe_distance_m
            })
    
    # Determine overall state
    if hammer_detected:
        return {"state": "HAMMER_SUPPRESS", "violations": violations}
    elif person_detected:
        return {"state": "ALERT_REAR", "violations": violations}
    else:
        return {"state": "SAFE", "violations": violations}

# ---- EASY HAND-TEST PRESETS (feel free to tune) ----
# These override the detector thresholds after we create it
TEST_THRESHOLDS = dict(
    # Kite field parameters
    KITE_MAX_ANGLE=20.0715,
    KITE_MAX_DISTANCE=212.932,
    KITE_CENTER_DISTANCE=250.0,
    HAMMER_MAX_DISTANCE=100.0,
    # Dwell short so your hand movement is responsive
    DWELL_MS=100,
    MIN_POINTS_IN_ROI=1,   # Minimum points in kite field
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

# ---- Kite-shaped detection field parameters ----
KITE_MAX_ANGLE = 20.0715  # degrees
KITE_MAX_DISTANCE = 212.932  # cm at max angle
KITE_CENTER_DISTANCE = 250.0  # cm at 0 degrees
HAMMER_MAX_DISTANCE = 100.0  # cm - anything closer is considered hammer

def calculate_kite_safe_distance(angle_deg):
    """
    Calculate the safe distance for a given angle in the kite-shaped field.
    Returns distance in meters.
    """
    # Clamp angle to kite field range
    angle_deg = max(-KITE_MAX_ANGLE, min(KITE_MAX_ANGLE, angle_deg))
    
    # Linear interpolation between center and edge
    # At 0°: 250cm, At ±20.0715°: 212.932cm
    ratio = abs(angle_deg) / KITE_MAX_ANGLE
    safe_distance_cm = KITE_CENTER_DISTANCE - (KITE_CENTER_DISTANCE - KITE_MAX_DISTANCE) * ratio
    
    return safe_distance_cm / 100.0  # Convert cm to meters

def is_in_kite_field(angle_deg):
    """Check if angle is within the kite detection field."""
    return abs(angle_deg) <= KITE_MAX_ANGLE

def print_kite_field_info():
    """Print kite field configuration for debugging."""
    print("Kite-shaped detection field:")
    print(f"  Field range: ±{KITE_MAX_ANGLE}°")
    print(f"  Safe distances:")
    for angle in [0, 10, 15, 20, -10, -15, -20]:
        if abs(angle) <= KITE_MAX_ANGLE:
            safe_dist = calculate_kite_safe_distance(angle)
            print(f"    {angle:3.0f}°: {safe_dist*100:6.1f}cm")
    print(f"  Hammer threshold: {HAMMER_MAX_DISTANCE}cm (any distance)")
    print()

# ---- Parser for sSN LMDscandata (ASCII/hex) ----
def parse_lmd(frame_text):
    """
    Extract all beams within the kite-shaped field from sSN LMDscandata.
    Returns [(angle_deg, distance_m)] or None if frame isn't LMDscandata.
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

    if step_deg <= 0:
        return None

    # Extract all points within the kite field
    points = []
    for idx in range(len(dist_hex)):
        angle_deg = start_deg + idx * step_deg
        
        # Only process points within the kite field
        if is_in_kite_field(angle_deg):
            try:
                d_mm = int(dist_hex[idx], 16)
                d_m = d_mm / 1000.0  # Convert mm to meters
                points.append((angle_deg, d_m))
            except ValueError:
                continue
    
    return points if points else None



def main():
    # Initialize kite field detection system
    print("=" * 60)
    print("SICK7 - TiM240 Kite-Shaped Safety Field")
    print("=" * 60)
    print_kite_field_info()

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
    last_raw_print = 0
    last_print = 0
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

                    # Analyze kite field for safety violations
                    out = analyze_kite_field(pts)
                    
                    # Optional: show raw data every 2 s for debugging
                    if pts and now_ms - last_raw_print >= 2000:
                        print(f"{time.strftime('%H:%M:%S')}  kite_field_points={len(pts)}")
                        # Show closest point for reference
                        if pts:
                            closest = min(pts, key=lambda x: x[1])
                            print(f"  closest: {closest[0]:.1f}° at {closest[1]:.3f}m")
                        last_raw_print = now_ms
                    
                    # Print status every 2 seconds or on state change
                    state = out.get("state")
                    violations = out.get("violations", [])
                    
                    if now_ms - last_print >= 2000:
                        print(f"{time.strftime('%H:%M:%S')}  state={state}  violations={len(violations)}")
                        if violations:
                            for v in violations[:3]:  # Show first 3 violations
                                print(f"  {v['type']}: {v['angle']:.1f}° at {v['distance']:.3f}m (safe: {v['safe_distance']:.3f}m)")
                        last_print = now_ms

                    # Print on state change
                    if state != last_state:
                        print(f"{time.strftime('%H:%M:%S')}  state={state}  violations={len(violations)}")
                        if violations:
                            for v in violations:
                                print(f"  {v['type']}: {v['angle']:.1f}° at {v['distance']:.3f}m (safe: {v['safe_distance']:.3f}m)")
                        last_state = state

            except socket.timeout:
                # no data this tick; continue waiting
                pass

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
