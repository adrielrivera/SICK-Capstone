#!/usr/bin/env python3
import time
import sys
import argparse
import serial
import pigpio

# ------------- TUNABLES -------------
SERIAL_PORT = "/dev/ttyACM0"   # e.g., /dev/ttyUSB0 on some boards
BAUD        = 115200
GPIO_PULSE  = 18               # BCM numbering
SAMPLES_PER_SEC = 800          # depends on your Arduino delay

# Envelope detection
ENVELOPE_ALPHA     = 0.12      # 0<alpha<1; lower = more smoothing
TRIGGER_THRESHOLD  = 60        # counts above baseline (0–1023 scale)
CAPTURE_MS         = 250       # how long to hunt for peak after trigger
REFRACTORY_MS      = 200       # ignore re-triggers after a hit

# Amplitude → pulse width mapping (linear)
# Calibrate these to taste:
A_MIN      = 80                # counts; small tap baseline
A_MAX      = 700               # counts; huge slam
W_MIN_MS   = 60                # ms
W_MAX_MS   = 1500              # ms

# Optional: decay baseline slowly (helps auto-center)
BASELINE_ALPHA     = 0.001

# ------------- HELPERS -------------
def clamp(x, lo, hi):
    return max(lo, min(hi, x))

def map_linear(x, x0, x1, y0, y1):
    if x1 <= x0:
        return y0
    t = (x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)

def build_pulse_wave(pi, gpio, width_ms):
    """Use pigpio waves for accurate pulse."""
    pi.write(gpio, 0)
    pi.wave_clear()

    up = pigpio.pulse(gpio_on=1<<gpio, gpio_off=0, delay=int(width_ms*1000))
    dn = pigpio.pulse(gpio_on=0,       gpio_off=1<<gpio, delay=1)  # ensure low

    pi.wave_add_generic([up, dn])
    wid = pi.wave_create()
    return wid

# ------------- MAIN -------------
def main():
    ap = argparse.ArgumentParser(description="PBT amplitude → pulse on Raspberry Pi (serial from Arduino).")
    ap.add_argument("--port", default=SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--gpio", type=int, default=GPIO_PULSE)
    ap.add_argument("--print-peaks", action="store_true", help="Print peak values & pulse widths.")
    args = ap.parse_args()

    # pigpio init
    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: pigpio daemon not running. Run: sudo systemctl start pigpiod", file=sys.stderr)
        sys.exit(1)
    pi.set_mode(args.gpio, pigpio.OUTPUT)
    pi.write(args.gpio, 0)

    # serial init
    ser = serial.Serial(args.port, args.baud, timeout=1)
    # Let serial settle
    time.sleep(0.2); ser.reset_input_buffer()

    # State
    baseline = 0.0
    env = 0.0
    armed = True
    peak = 0.0
    cap_end = 0.0
    last_fire = 0.0

    # Quick baseline warm-up (200 ms)
    t0 = time.time()
    warm_n = 0
    while time.time() - t0 < 0.2:
        line = ser.readline().decode(errors="ignore").strip()
        if not line: 
            continue
        try:
            raw = int(line)
        except ValueError:
            continue
        baseline = (baseline*warm_n + raw) / (warm_n + 1) if warm_n > 0 else raw
        warm_n += 1

    print(f"Baseline ≈ {baseline:.1f} (0–1023 ADC counts)")

    # Loop
    try:
        sample_period = 1.0 / SAMPLES_PER_SEC if SAMPLES_PER_SEC > 0 else 0.0
        next_tick = time.time()

        while True:
            line = ser.readline().decode(errors="ignore").strip()
            if not line:
                # keep loop cadence even if missing samples
                if sample_period:
                    now = time.time()
                    if now < next_tick:
                        time.sleep(next_tick - now)
                    next_tick += sample_period
                continue

            try:
                raw = int(line)
            except ValueError:
                continue

            # Update baseline slowly so it follows drift but ignores hits
            baseline = (1.0 - BASELINE_ALPHA) * baseline + BASELINE_ALPHA * raw

            # Full-wave rectify around baseline → envelope IIR
            x = abs(raw - baseline)
            env = (1.0 - ENVELOPE_ALPHA) * env + ENVELOPE_ALPHA * x

            now = time.time()

            if armed:
                if env > TRIGGER_THRESHOLD:
                    armed = False
                    peak = env
                    cap_end = now + (CAPTURE_MS / 1000.0)
            else:
                # During capture window track the maximum envelope
                if env > peak:
                    peak = env

                # End conditions: time up or envelope dropped a lot
                if now >= cap_end or env < (TRIGGER_THRESHOLD * 0.5):
                    # Determine pulse width (linear map with clamp)
                    a_clamped = clamp(peak, A_MIN, A_MAX)
                    width_ms = map_linear(a_clamped, A_MIN, A_MAX, W_MIN_MS, W_MAX_MS)
                    width_ms = clamp(width_ms, W_MIN_MS, W_MAX_MS)

                    if args.print_peaks:
                        print(f"Peak={peak:.1f} → {width_ms:.0f} ms")

                    # Fire pulse precisely
                    wid = build_pulse_wave(pi, args.gpio, width_ms)
                    if wid >= 0:
                        pi.wave_send_once(wid)
                        while pi.wave_tx_busy():
                            time.sleep(0.001)
                        pi.wave_delete(wid)
                    else:
                        # Fallback (shouldn't happen)
                        pi.write(args.gpio, 1); time.sleep(width_ms/1000.0); pi.write(args.gpio, 0)

                    last_fire = now
                    # Refractory
                    time.sleep(REFRACTORY_MS / 1000.0)
                    armed = True

            # Soft pace (depends on Arduino rate)
            if sample_period:
                now = time.time()
                if now < next_tick:
                    time.sleep(next_tick - now)
                next_tick += sample_period

    except KeyboardInterrupt:
        pass
    finally:
        pi.write(args.gpio, 0)
        pi.stop()
        ser.close()

if __name__ == "__main__":
    main()
