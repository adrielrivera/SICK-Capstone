import time, math, collections, random

# config (tune later)
SCAN_HZ = 15
ROI_MIN_DEG, ROI_MAX_DEG = -80.0, 80.0
D_CLOSE = 0.40                # hammer occlusion threshold (m)
D_PERSON_MIN, D_PERSON_MAX = 0.60, 3.00
DWELL_MS = 120
LOCKOUT_MS = 1000
MISSING_SCAN_TIMEOUT_MS = 300
MIN_POINTS_IN_ROI = 8         # avoid noise glitches
EMA_ALPHA = 0.4               # light smoothing on distance

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

    def update(self, points, now_ms):
        self.last_scan_ts = now_ms

        dmin, count = min_distance(points)
        if count < MIN_POINTS_IN_ROI or dmin is None:
            return {"state":"FAILSAFE", "reason":"insufficient_points"}

        # smooth min distance a bit
        if self.ema_min is None:
            self.ema_min = dmin
        else:
            self.ema_min = EMA_ALPHA*dmin + (1-EMA_ALPHA)*self.ema_min

        in_person = D_PERSON_MIN <= self.ema_min <= D_PERSON_MAX
        is_hammer = self.ema_min < D_CLOSE

        # dwell timers
        if in_person:
            self.person_since_ms = self.person_since_ms or now_ms
        else:
            self.person_since_ms = None

        if is_hammer:
            self.hammer_since_ms = self.hammer_since_ms or now_ms
        else:
            self.hammer_since_ms = None

        # lockout after hammer occlusion
        in_lockout = (now_ms - self.last_alert_ts) < LOCKOUT_MS
        hammer_dwelled = self.hammer_since_ms and (now_ms - self.hammer_since_ms >= DWELL_MS)
        person_dwelled = self.person_since_ms and (now_ms - self.person_since_ms >= DWELL_MS)

        if hammer_dwelled:
            # treat as hammer; suppress alerts
            self.last_alert_ts = now_ms  # start lockout
            return {"state":"HAMMER_SUPPRESS", "dmin":round(self.ema_min,3)}

        if person_dwelled and not in_lockout:
            self.last_alert_ts = now_ms
            return {"state":"ALERT_REAR", "dmin":round(self.ema_min,3)}

        return {"state":"SAFE", "dmin":round(self.ema_min,3)}

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
            out = det.update(pts, now_ms)
            wd = det.watchdog(now_ms)
            if wd: out = wd
            print(mode, out)
            time.sleep(1.0/SCAN_HZ)
        ts0 = time.time()
