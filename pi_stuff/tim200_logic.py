import time, math, collections, random

# config (tune later)
SCAN_HZ = 15
ROI_MIN_DEG, ROI_MAX_DEG = -80.0, 80.0
D_CLOSE = 0.40                # hammer occlusion threshold (m)
D_PERSON_MIN, D_PERSON_MAX = 0.60, 3.00
DWELL_MS = 120
SIM_DWELL_MS = 2  # Reduced dwell time for simulation (2 scans = ~133ms at 15Hz)
LOCKOUT_MS = 1000
MISSING_SCAN_TIMEOUT_MS = 300
MIN_POINTS_IN_ROI = 8         # avoid noise glitches
EMA_ALPHA = 0.15              # heavier smoothing to reduce fluctuations
HYSTERESIS_MARGIN = 0.08      # add hysteresis to prevent flickering
CONFIDENCE_THRESHOLD = 2      # require 2 consecutive readings to change state
CONFIDENCE_DECAY_THRESHOLD = 5  # require 5 consecutive non-detections to reset timer

# utilities
def in_roi(pt):
    ang, dist = pt
    return ROI_MIN_DEG <= ang <= ROI_MAX_DEG and dist > 0

def min_distance(points):
    dmin = float("inf")
    n = 0
    for a,d in points:
        if in_roi((a,d)):
            dmin = min(dmin, d)
            n += 1
    return (dmin if n else None), n

# simple scan generator for development (replace with real LiDAR later)
def get_scan(sim_mode="idle", t=0.0):
    # returns list[(angle_deg, distance_m)]
    pts = []
    for i in range(-100, 101):  # -100..+100 deg
        ang = i * 0.8  # ~160 deg FOV
        base = 5.0     # empty far background
        dist = base

        if sim_mode == "idle":
            pass

        elif sim_mode == "hammer":
            # near occlusion wedge under sensor
            if -20 <= ang <= 20:
                dist = 0.30 + 0.02*random.random()

        elif sim_mode == "person":
            # person at ~1.4 m approaching to 0.9 m
            approach = 1.4 - 0.5*math.sin(min(1.0, t/2.0)*math.pi/2)
            if -30 <= ang <= 30:
                dist = approach + 0.05*random.random()

        # add mild noise
        dist += 0.01*random.random()
        pts.append((ang, dist))
    return pts

# detector
class HammerRearDetector:
    def __init__(self):
        self.last_alert_ts = -1e9
        self.person_since_ms = None
        self.hammer_since_ms = None
        self.ema_min = None
        self.last_scan_ts = None
        self.current_state = "SAFE"  # track current state for hysteresis
        self.person_confidence = 0   # confidence counter for person detection
        self.hammer_confidence = 0   # confidence counter for hammer detection

    def update(self, points, now_ms, sim_mode="unknown"):
        self.last_scan_ts = now_ms

        dmin, count = min_distance(points)
        if count < MIN_POINTS_IN_ROI or dmin is None:
            return {"state":"FAILSAFE", "reason":"insufficient_points"}

        # smooth min distance a bit
        if self.ema_min is None:
            self.ema_min = dmin
        else:
            self.ema_min = EMA_ALPHA*dmin + (1-EMA_ALPHA)*self.ema_min
        
        # Store simulation mode for context-aware detection
        # Reset state when simulation mode changes
        if hasattr(self, 'sim_mode') and self.sim_mode != sim_mode:
            self.current_state = "SAFE"
            self.person_since_ms = None
            self.hammer_since_ms = None
        self.sim_mode = sim_mode

        # Context-aware detection with hysteresis
        if self.sim_mode == "hammer":
            # During hammer simulation, only detect hammer, not person
            in_person = False
            is_hammer = self.ema_min < D_CLOSE
        elif self.sim_mode == "person":
            # During person simulation, only detect person, not hammer
            if self.current_state == "SAFE":
                in_person = D_PERSON_MIN <= self.ema_min <= D_PERSON_MAX
            else:
                # Use hysteresis to maintain ALERT_REAR state - expand the range to stay in alert
                in_person = (D_PERSON_MIN - HYSTERESIS_MARGIN) <= self.ema_min <= (D_PERSON_MAX + HYSTERESIS_MARGIN)
            is_hammer = False
        else:
            # Normal detection mode (idle or unknown)
            if self.current_state == "SAFE":
                in_person = D_PERSON_MIN <= self.ema_min <= D_PERSON_MAX
                is_hammer = self.ema_min < D_CLOSE
            elif self.current_state == "ALERT_REAR":
                # Use lower threshold to exit person state (hysteresis)
                in_person = (D_PERSON_MIN - HYSTERESIS_MARGIN) <= self.ema_min <= (D_PERSON_MAX + HYSTERESIS_MARGIN)
                is_hammer = self.ema_min < (D_CLOSE - HYSTERESIS_MARGIN)
            elif self.current_state == "HAMMER_SUPPRESS":
                # Use higher threshold to exit hammer state (hysteresis)
                in_person = (D_PERSON_MIN + HYSTERESIS_MARGIN) <= self.ema_min <= (D_PERSON_MAX - HYSTERESIS_MARGIN)
                is_hammer = self.ema_min < (D_CLOSE + HYSTERESIS_MARGIN)
            else:
                in_person = D_PERSON_MIN <= self.ema_min <= D_PERSON_MAX
                is_hammer = self.ema_min < D_CLOSE

        # Completely rewritten simple and reliable logic
        # Use simulation-appropriate dwell time
        dwell_ms = SIM_DWELL_MS if self.sim_mode in ["person", "hammer"] else DWELL_MS
        
        # Initialize new_state
        new_state = "SAFE"
        
        # Handle each simulation mode explicitly
        if self.sim_mode == "hammer":
            # Hammer simulation: only detect hammer, never person
            if is_hammer:
                if self.hammer_since_ms is None:
                    self.hammer_since_ms = now_ms
                elif (now_ms - self.hammer_since_ms >= dwell_ms):
                    new_state = "HAMMER_SUPPRESS"
            else:
                self.hammer_since_ms = None
                
        elif self.sim_mode == "person":
            # Person simulation: only detect person, never hammer
            if in_person:
                if self.person_since_ms is None:
                    self.person_since_ms = now_ms
                elif (now_ms - self.person_since_ms >= dwell_ms):
                    new_state = "ALERT_REAR"
            else:
                self.person_since_ms = None
                
        else:
            # Normal mode (idle or unknown): detect both person and hammer
            if is_hammer:
                if self.hammer_since_ms is None:
                    self.hammer_since_ms = now_ms
                elif (now_ms - self.hammer_since_ms >= dwell_ms):
                    new_state = "HAMMER_SUPPRESS"
            elif in_person:
                if self.person_since_ms is None:
                    self.person_since_ms = now_ms
                elif (now_ms - self.person_since_ms >= dwell_ms):
                    new_state = "ALERT_REAR"
            else:
                # Reset timers if nothing detected
                self.person_since_ms = None
                self.hammer_since_ms = None
        
        # Update current state
        self.current_state = new_state
        
        # Debug output for idle mode false alarms
        if self.sim_mode == "idle" and new_state == "ALERT_REAR":
            print(f"DEBUG IDLE: dmin={self.ema_min:.3f}, in_person={in_person}, is_hammer={is_hammer}, person_since_ms={self.person_since_ms}")
        
        return {"state": new_state, "dmin": round(self.ema_min, 3)}

    def watchdog(self, now_ms):
        if self.last_scan_ts is None or (now_ms - self.last_scan_ts) > MISSING_SCAN_TIMEOUT_MS:
            return {"state":"FAILSAFE", "reason":"missing_scans"}
        return None

# quick bench run
if __name__ == "__main__":
    det = HammerRearDetector()
    modes = [("idle", 1.5), ("hammer", 1.0), ("idle", 0.6), ("person", 2.5)]
    ts0 = time.time()

    for mode, duration in modes:
        while time.time() - ts0 < duration:
            t = time.time() - ts0
            pts = get_scan(mode, t)
            now_ms = int(time.time()*1000)
            out = det.update(pts, now_ms, mode)
            wd = det.watchdog(now_ms)
            if wd: out = wd
            print(mode, out)
            time.sleep(1.0/SCAN_HZ)
        ts0 = time.time()
