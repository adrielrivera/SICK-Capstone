# TiM200 rear-zone classifier (single-point, interactive)
# Enter: angle_deg distance_m   e.g., 10 0.32
# Quit:  q

# ---- Tunables (set to your current plan) ----
ROI_MIN_DEG = -80.0      # rear sector (relative to straight-down axis)
ROI_MAX_DEG =  80.0
D_CLOSE      = 0.40      # < D_CLOSE => treat as hammer occlusion
D_PERSON_MIN = 0.60      # person range lower bound
D_PERSON_MAX = 3.00      # person range upper bound

def classify(angle_deg: float, distance_m: float) -> str:
    """Return one of: 'OUT_OF_ROI', 'HAMMER', 'PERSON', 'SAFE'."""
    # invalid/empty returns
    if distance_m <= 0:
        return "SAFE"

    # region of interest: behind the user
    if not (ROI_MIN_DEG <= angle_deg <= ROI_MAX_DEG):
        return "OUT_OF_ROI"

    # hammer vs person vs safe (by distance)
    if distance_m < D_CLOSE:
        return "HAMMER"   # very close occlusion (likely the hammer)
    if D_PERSON_MIN <= distance_m <= D_PERSON_MAX:
        return "PERSON"   # plausible person approach range
    return "SAFE"

def main():
    print("TiM200 rear detector — enter 'angle_deg distance_m' (e.g., 12 1.4). Type 'q' to quit.")
    print(f"ROI=({ROI_MIN_DEG}..{ROI_MAX_DEG}) deg, HAMMER if d<{D_CLOSE} m, PERSON if {D_PERSON_MIN}..{D_PERSON_MAX} m.\n")

    while True:
        s = input("> ").strip()
        if s.lower() in ("q", "quit", "exit"):
            break
        try:
            ang_str, dist_str = s.split()
            ang = float(ang_str)
            dist = float(dist_str)
        except Exception:
            print("  Format: <angle_deg> <distance_m>   e.g., 15 0.32")
            continue

        verdict = classify(ang, dist)
        print(f"  angle={ang:.2f}°, distance={dist:.3f} m  →  {verdict}")

if __name__ == "__main__":
    main()
