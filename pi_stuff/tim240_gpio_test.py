#!/usr/bin/env python3
"""
Simplified TiM240 GPIO Test
Tests if TiM240 correctly outputs detection signal to GPIO 17 (Pin 11)
No Arduino required - only tests TiM240 -> Pi -> GPIO 17
"""
import socket
import time
import math
import RPi.GPIO as GPIO

# GPIO pin for person detection output (3.3V)
DETECTION_GPIO_PIN = 17  # GPIO17 (Physical Pin 11) - Changed from GPIO 18

# LiDAR connection
HOST, PORT = "192.168.0.20", 2111
STX, ETX = b"\x02", b"\x03"

# Kite-shaped detection field parameters
KITE_MAX_ANGLE = 20.0715  # degrees
KITE_MAX_DISTANCE = 180.0  # cm at max angle (shadow distance on floor) - Decreased to prevent floor detection (more lenient)
KITE_CENTER_DISTANCE = 200.0  # cm at 0 degrees (shadow distance on floor) - Decreased to prevent floor detection (more lenient)
HAMMER_MAX_DISTANCE = 200.0  # cm - anything closer is considered hammer
LIDAR_HEIGHT = 280.0  # cm above arcade base
LIDAR_DOWN_ANGLE = 41.8  # degrees from vertical

def send(sock, cmd, expect_reply=True, timeout=2.0):
    sock.settimeout(timeout)
    sock.sendall(STX + cmd.encode("ascii") + ETX)
    if not expect_reply:
        return b""
    try:
        return sock.recv(65535)
    except socket.timeout:
        return b""

def calculate_kite_safe_distance(angle_deg):
    """Calculate the safe distance for a given angle in the kite-shaped field."""
    angle_deg = max(-KITE_MAX_ANGLE, min(KITE_MAX_ANGLE, angle_deg))
    ratio = abs(angle_deg) / KITE_MAX_ANGLE
    shadow_distance_cm = KITE_CENTER_DISTANCE - (KITE_CENTER_DISTANCE - KITE_MAX_DISTANCE) * ratio
    actual_beam_distance_cm = shadow_distance_cm / math.sin(math.radians(LIDAR_DOWN_ANGLE))
    return actual_beam_distance_cm / 100.0  # Convert cm to meters

def is_in_kite_field(angle_deg):
    """Check if angle is within the kite detection field."""
    return abs(angle_deg) <= KITE_MAX_ANGLE

def parse_lmd(frame_text):
    """Extract all beams within the kite-shaped field from sSN LMDscandata."""
    line = " ".join(frame_text.strip().split())
    if not line.startswith("sSN LMDscandata"):
        return None

    toks = line.split()
    try:
        i = toks.index("DIST1")
    except ValueError:
        return None

    def h2s32(h):
        v = int(h, 16)
        return v - (1 << 32) if v & (1 << 31) else v

    try:
        scale = int(toks[i+1], 16)
        scale_offset = int(toks[i+2], 16)
        start_deg = h2s32(toks[i+3]) / 10000.0
        step_deg = int(toks[i+4], 16) / 10000.0
        count = int(toks[i+5], 16)
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
        if is_in_kite_field(angle_deg):
            try:
                d_mm = int(dist_hex[idx], 16)
                d_m = d_mm / 1000.0  # Convert mm to meters
                points.append((angle_deg, d_m))
            except ValueError:
                continue
    
    return points if points else None

def analyze_kite_field(points):
    """Analyze points within the kite-shaped field for safety violations."""
    if not points:
        return {"state": "FAILSAFE", "reason": "no_points", "violations": []}
    
    violations = []
    hammer_detected = False
    person_detected = False
    
    for angle_deg, distance_m in points:
        safe_distance_m = calculate_kite_safe_distance(angle_deg)
        
        # Check for hammer (very close objects)
        if distance_m < (HAMMER_MAX_DISTANCE / 100.0):
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

def main():
    # Initialize GPIO
    gpio_working = False
    try:
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(DETECTION_GPIO_PIN, GPIO.OUT)
        GPIO.output(DETECTION_GPIO_PIN, GPIO.LOW)  # Start with no detection
        gpio_working = True
        print("✅ GPIO initialized successfully")
        print(f"   GPIO Pin: {DETECTION_GPIO_PIN} (Physical Pin 11)")
    except RuntimeError as e:
        print(f"⚠️  GPIO initialization failed: {e}")
        print("   Try running with: sudo python3 tim240_gpio_test.py")
        print("   Or add user to gpio group: sudo usermod -a -G gpio $USER")
        return
    except Exception as e:
        print(f"⚠️  GPIO error: {e}")
        return

    # Connect to LiDAR
    print(f"\n{'='*60}")
    print("TiM240 GPIO Test - Detection Output to GPIO 17 (Pin 11)")
    print(f"{'='*60}")
    print(f"Connecting to TiM240 at {HOST}:{PORT}")
    
    try:
        s = socket.create_connection((HOST, PORT), timeout=3)
    except Exception as e:
        print(f"❌ Failed to connect to TiM240: {e}")
        print("   Check that TiM240 is powered and IP address is correct")
        GPIO.cleanup()
        return

    # Sanity check
    ident = send(s, "sRN DeviceIdent").decode(errors="ignore").strip()
    print(f"Device Ident: {ident if ident else '(no reply)'}")

    # Enable stream
    print("Enabling LiDAR stream...")
    send(s, "sEN LMDscandata 1", expect_reply=False)

    # Stream loop
    buf = b""
    last_state = None
    last_print = 0
    s.settimeout(1.0)

    print("\n" + "="*60)
    print("Monitoring TiM240 detection...")
    print("  GPIO 17 will be HIGH when person detected")
    print("  GPIO 17 will be LOW when area is clear")
    print("  Press Ctrl+C to stop")
    print("="*60 + "\n")

    try:
        while True:
            try:
                chunk = s.recv(65535)
                if not chunk:
                    continue
                buf += chunk

                # Split by ETX; frames are STX...ETX
                while ETX in buf:
                    frame, buf = buf.split(ETX, 1)
                    if STX in frame:
                        frame = frame.split(STX)[-1]
                    text = frame.decode("ascii", errors="ignore")
                    pts = parse_lmd(text)
                    if pts is None:
                        continue

                    now_ms = int(time.time() * 1000)

                    # Analyze kite field for safety violations
                    out = analyze_kite_field(pts)
                    state = out.get("state")
                    violations = out.get("violations", [])

                    # Control GPIO based on person detection
                    # HAMMER_SUPPRESS should NOT trigger GPIO HIGH - only ALERT_REAR should
                    person_detected = (state == "ALERT_REAR")
                    GPIO.output(DETECTION_GPIO_PIN, GPIO.HIGH if person_detected else GPIO.LOW)

                    # Print status every 2 seconds or on state change
                    if state != last_state or (now_ms - last_print >= 2000):
                        gpio_status = "HIGH" if person_detected else "LOW"
                        print(f"{time.strftime('%H:%M:%S')}  state={state:20}  violations={len(violations):2}  GPIO={gpio_status:4}")
                        
                        if violations:
                            for v in violations[:3]:  # Show first 3 violations
                                print(f"  {v['type']}: {v['angle']:.1f}° at {v['distance']:.3f}m (safe: {v['safe_distance']:.3f}m)")
                        
                        # Enhanced output on state change
                        if state != last_state:
                            if state == "ALERT_REAR":
                                print(f"\n🚨 TiM240 PERSON DETECTED - GPIO 17 = HIGH")
                                print(f"   Time: {time.strftime('%H:%M:%S')}")
                                print(f"   State: {state}")
                                print(f"   Violations: {len(violations)}")
                            elif state == "HAMMER_SUPPRESS":
                                print(f"\n🔨 TiM240 HAMMER DETECTED - SUPPRESSED (GPIO 17 = LOW)")
                                print(f"   Time: {time.strftime('%H:%M:%S')}")
                                print(f"   State: {state} - No alarm triggered")
                                print(f"   Violations: {len(violations)}")
                            elif state == "SAFE" and last_state in ["ALERT_REAR", "HAMMER_SUPPRESS"]:
                                print(f"\n✅ TiM240 CLEAR - GPIO 17 = LOW")
                                print(f"   Time: {time.strftime('%H:%M:%S')}")
                        
                        last_state = state
                        last_print = now_ms

            except socket.timeout:
                pass

    except KeyboardInterrupt:
        print("\n\nStopping...")
    finally:
        print("\nDisabling stream and closing...")
        try:
            send(s, "sEN LMDscandata 0", expect_reply=False)
        except Exception:
            pass
        s.close()
        
        # Cleanup GPIO
        GPIO.output(DETECTION_GPIO_PIN, GPIO.LOW)
        GPIO.cleanup()
        print("✅ GPIO cleaned up and set to LOW")

if __name__ == "__main__":
    main()

