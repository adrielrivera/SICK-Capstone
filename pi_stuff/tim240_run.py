#!/usr/bin/env python3
import socket, time, math, serial
import RPi.GPIO as GPIO

# ---- Multi-LiDAR Safety System ----
# TiM240: Ethernet -> Pi (rear detection)
# TiM100/TiM150: Arduino -> Pi (left/right detection)

# Individual LiDAR status tracking
tim100_detected = False
tim150_detected = False
last_tim1xx_update = 0

# GPIO pin for person detection output (3.3V)
DETECTION_GPIO_PIN = 18  # GPIO18 (Pin 12)

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

# ---- Arduino connection for safety system ----
ARDUINO_PORT = "/dev/ttyUSB0"  # Adjust as needed
ARDUINO_BAUD = 115200
arduino_ser = None

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
KITE_MAX_DISTANCE = 212.932  # cm at max angle (shadow distance on floor)
KITE_CENTER_DISTANCE = 250.0  # cm at 0 degrees (shadow distance on floor)
HAMMER_MAX_DISTANCE = 100.0  # cm - anything closer is considered hammer
LIDAR_HEIGHT = 280.0  # cm above arcade base
LIDAR_DOWN_ANGLE = 41.8  # degrees from vertical

def calculate_kite_safe_distance(angle_deg):
    """
    Calculate the safe distance for a given angle in the kite-shaped field.
    Returns actual beam distance in meters using 3D geometry.
    """
    # Clamp angle to kite field range
    angle_deg = max(-KITE_MAX_ANGLE, min(KITE_MAX_ANGLE, angle_deg))
    
    # Linear interpolation between center and edge for shadow distances
    # At 0°: 250cm, At ±20.0715°: 212.932cm
    ratio = abs(angle_deg) / KITE_MAX_ANGLE
    shadow_distance_cm = KITE_CENTER_DISTANCE - (KITE_CENTER_DISTANCE - KITE_MAX_DISTANCE) * ratio
    
    # Convert shadow distance to actual beam distance using 3D geometry
    # actual_beam_distance = shadow_distance / sin(41.8°)
    actual_beam_distance_cm = shadow_distance_cm / math.sin(math.radians(LIDAR_DOWN_ANGLE))
    
    return actual_beam_distance_cm / 100.0  # Convert cm to meters

def is_in_kite_field(angle_deg):
    """Check if angle is within the kite detection field."""
    return abs(angle_deg) <= KITE_MAX_ANGLE

def print_kite_field_info():
    """Print kite field configuration for debugging."""
    print("Kite-shaped detection field:")
    print(f"  Field range: ±{KITE_MAX_ANGLE}°")
    print(f"  LiDAR height: {LIDAR_HEIGHT}cm, down angle: {LIDAR_DOWN_ANGLE}°")
    print(f"  Safe distances (actual beam distances):")
    for angle in [0, 10, 15, 20, -10, -15, -20]:
        if abs(angle) <= KITE_MAX_ANGLE:
            safe_dist = calculate_kite_safe_distance(angle)
            # Calculate shadow distance for reference
            ratio = abs(angle) / KITE_MAX_ANGLE
            shadow_dist = KITE_CENTER_DISTANCE - (KITE_CENTER_DISTANCE - KITE_MAX_DISTANCE) * ratio
            print(f"    {angle:3.0f}°: {safe_dist*100:6.1f}cm beam (shadow: {shadow_dist:6.1f}cm)")
    print(f"  Hammer threshold: {HAMMER_MAX_DISTANCE}cm actual beam distance")
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



def init_arduino_connection():
    """Initialize Arduino connection for safety system."""
    global arduino_ser
    try:
        arduino_ser = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        time.sleep(0.2)
        arduino_ser.reset_input_buffer()
        print(f"Arduino connected on {ARDUINO_PORT} @ {ARDUINO_BAUD} baud")
        return True
    except Exception as e:
        print(f"WARNING: Could not connect to Arduino on {ARDUINO_PORT}: {e}")
        print("Safety system will run without Arduino integration")
        return False

def send_arduino_trigger():
    """Send safety trigger to Arduino."""
    if arduino_ser and not arduino_ser.closed:
        try:
            # Send a simple trigger signal - Arduino will handle the alarm logic
            arduino_ser.write(b"LIDAR_TRIGGER\n")
            arduino_ser.flush()
            print("  LiDAR trigger sent to Arduino")
        except Exception as e:
            print(f"  Error sending LiDAR trigger to Arduino: {e}")


def read_arduino_messages():
    """Read and display Arduino status messages."""
    global arduino_ser
    if arduino_ser and not arduino_ser.closed:
        try:
            while arduino_ser.in_waiting > 0:
                line = arduino_ser.readline().decode(errors="ignore").strip()
                if line and line.startswith("#"):
                    print(f"  Arduino: {line}")
        except Exception as e:
            pass  # Ignore serial read errors

def read_tim1xx_status():
    """Read TiM1xx status from Arduino."""
    global tim100_detected, tim150_detected, last_tim1xx_update, arduino_ser
    
    if arduino_ser and not arduino_ser.closed:
        try:
            # Debug: Check if there's data waiting
            if arduino_ser.in_waiting > 0:
                print(f"DEBUG: {arduino_ser.in_waiting} bytes waiting from Arduino")
            
            while arduino_ser.in_waiting > 0:
                line = arduino_ser.readline().decode(errors="ignore").strip()
                
                # Debug: Print ALL Arduino messages
                if line:
                    print(f"DEBUG Arduino: '{line}'")
                
                # Parse individual detection messages
                if "TiM100 DETECTED" in line:
                    tim100_detected = True
                    last_tim1xx_update = time.time()
                    print(f"\n🚨 TiM100 DETECTED - Person on LEFT side")
                elif "TiM100 CLEAR" in line:
                    tim100_detected = False
                    last_tim1xx_update = time.time()
                    print(f"\n✅ TiM100 CLEAR - LEFT side clear")
                elif "TiM150 DETECTED" in line:
                    tim150_detected = True
                    last_tim1xx_update = time.time()
                    print(f"\n🚨 TiM150 DETECTED - Person on RIGHT side")
                elif "TiM150 CLEAR" in line:
                    tim150_detected = False
                    last_tim1xx_update = time.time()
                    print(f"\n✅ TiM150 CLEAR - RIGHT side clear")
                elif line.startswith("# LIDAR_STATUS:"):
                    # Parse: "TIM100=DETECTED TIM150=CLEAR"
                    parts = line.split()
                    if len(parts) >= 4:
                        tim100_status = "DETECTED" in parts[1]
                        tim150_status = "DETECTED" in parts[3]
                        
                        # Only update if status changed
                        if tim100_detected != tim100_status or tim150_detected != tim150_status:
                            tim100_detected = tim100_status
                            tim150_detected = tim150_status
                            last_tim1xx_update = time.time()
                            
                            # Enhanced terminal output
                            print(f"\n{'='*60}")
                            print(f"🔍 LiDAR STATUS UPDATE - {time.strftime('%H:%M:%S')}")
                            print(f"   TiM100 (Left):  {'🚨 DETECTED' if tim100_detected else '✅ CLEAR'}")
                            print(f"   TiM150 (Right): {'🚨 DETECTED' if tim150_detected else '✅ CLEAR'}")
                            print(f"{'='*60}\n")
        except Exception as e:
            print(f"DEBUG Serial Error: {e}")
            pass  # Ignore serial read errors
    else:
        print("DEBUG: Arduino not connected or closed")

def get_combined_safety_status(state):
    """Get combined safety status from all LiDARs."""
    global tim100_detected, tim150_detected
    
    # Get TiM240 status (from existing state variable)
    tim240_alert = (state == "ALERT_REAR")
    
    # Determine combined status
    if tim240_alert or tim100_detected or tim150_detected:
        unsafe_areas = []
        if tim240_alert: unsafe_areas.append("REAR")
        if tim100_detected: unsafe_areas.append("LEFT")
        if tim150_detected: unsafe_areas.append("RIGHT")
        
        return "DANGER", {
            'rear': tim240_alert,
            'left': tim100_detected,
            'right': tim150_detected,
            'areas': unsafe_areas
        }
    else:
        return "SAFE", {
            'rear': False,
            'left': False,
            'right': False,
            'areas': []
        }

def main():
    # Initialize GPIO
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(DETECTION_GPIO_PIN, GPIO.OUT)
    GPIO.output(DETECTION_GPIO_PIN, GPIO.LOW)  # Start with no detection
    
    # Initialize kite field detection system
    print("=" * 60)
    print("SICK7 - TiM240 Kite-Shaped Safety Field")
    print("=" * 60)
    print_kite_field_info()
    print(f"GPIO Detection Output: Pin {DETECTION_GPIO_PIN} (3.3V when person detected)")

    # Initialize Arduino connection
    arduino_connected = init_arduino_connection()

    # Connect to LiDAR
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
    last_status_request = 0
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

                    # Read Arduino messages (credit tracking, status updates)
                    read_arduino_messages()
                    
                    # Read TiM1xx status from Arduino
                    read_tim1xx_status()
                    
                    # Request Arduino status every 5 seconds
                    if now_ms - last_status_request >= 5000:
                        if arduino_ser and not arduino_ser.closed:
                            try:
                                arduino_ser.write(b"STATUS\n")
                                arduino_ser.flush()
                                last_status_request = now_ms
                            except Exception as e:
                                pass
                    
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
                    
                    # Control GPIO based on person detection
                    person_detected = (state == "ALERT_REAR")
                    GPIO.output(DETECTION_GPIO_PIN, GPIO.HIGH if person_detected else GPIO.LOW)
                    
                    if now_ms - last_print >= 2000:
                        print(f"{time.strftime('%H:%M:%S')}  state={state}  violations={len(violations)}  GPIO={'HIGH' if person_detected else 'LOW'}")
                        if violations:
                            for v in violations[:3]:  # Show first 3 violations
                                print(f"  {v['type']}: {v['angle']:.1f}° at {v['distance']:.3f}m (safe: {v['safe_distance']:.3f}m)")
                        last_print = now_ms

                    # Print on state change and handle safety triggers
                    if state != last_state:
                        print(f"{time.strftime('%H:%M:%S')}  state={state}  violations={len(violations)}")
                        
                        # Enhanced output for TiM240 detection
                        if state == "ALERT_REAR":
                            print(f"\n🚨 ===== TiM240 PERSON DETECTED - REAR SIDE =====")
                            print(f"   Time: {time.strftime('%H:%M:%S')}")
                            print(f"   Sensor: TiM240 (Rear)")
                            print(f"   Violations: {len(violations)}")
                            if violations:
                                for v in violations:
                                    print(f"   - {v['type']}: {v['angle']:.1f}° at {v['distance']:.3f}m (safe: {v['safe_distance']:.3f}m)")
                            print(f"🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨\n")
                        elif state == "SAFE" and last_state == "ALERT_REAR":
                            print(f"\n✅ ===== TiM240 AREA CLEAR - REAR SIDE =====")
                            print(f"   Time: {time.strftime('%H:%M:%S')}")
                            print(f"   Sensor: TiM240 (Rear)")
                            print(f"✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅✅\n")
                        
                        if violations:
                            for v in violations:
                                print(f"  {v['type']}: {v['angle']:.1f}° at {v['distance']:.3f}m (safe: {v['safe_distance']:.3f}m)")
                        
                        # Send safety trigger to Arduino if any LiDAR detects person
                        safety_status, safety_info = get_combined_safety_status(state)
                        if safety_status == "DANGER" and arduino_connected:
                            areas = safety_info['areas']
                            print(f"\n🚨🚨🚨 MULTI-LIDAR SAFETY ALERT 🚨🚨🚨")
                            print(f"   Time: {time.strftime('%H:%M:%S')}")
                            print(f"   Detected Areas: {', '.join(areas)}")
                            print(f"   Action: Game disabled, Alarm triggered")
                            print(f"🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨🚨\n")
                            send_arduino_trigger()
                        
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
        
        # Close Arduino connection
        if arduino_ser and not arduino_ser.closed:
            try:
                arduino_ser.close()
                print("Arduino connection closed")
            except Exception:
                pass
        
        # Cleanup GPIO
        GPIO.output(DETECTION_GPIO_PIN, GPIO.LOW)
        GPIO.cleanup()
        print("GPIO cleaned up")

if __name__ == "__main__":
    main()
