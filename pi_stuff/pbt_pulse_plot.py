#!/usr/bin/env python3
import time
import sys
import argparse
import serial
import pigpio
import matplotlib.pyplot as plt
from collections import deque

# configure ts boy
SERIAL_PORT = "/dev/ttyUSB0"   # keeps changing for some reason
BAUD        = 115200
GPIO_PULSE  = 18
SAMPLES_PER_SEC = 800

# envelope
ENVELOPE_ALPHA     = 0.12
TRIGGER_THRESHOLD  = 60
CAPTURE_MS         = 250
REFRACTORY_MS      = 200
BASELINE_ALPHA     = 0.001

# mapping
A_MIN, A_MAX       = 80, 700
W_MIN_MS, W_MAX_MS = 60, 1500

# plotting
N_PLOT     = 2000             # 2.5 s history at 800 SPS
PLOT_EVERY = 40               # update chart 20 Hz


def clamp(x, lo, hi): return max(lo, min(hi, x))

def map_linear(x, x0, x1, y0, y1):
    if x1 <= x0: return y0
    t = (x - x0) / (x1 - x0)
    return y0 + t * (y1 - y0)

def build_pulse_wave(pi, gpio, width_ms):
    pi.write(gpio, 0)
    pi.wave_clear()
    up = pigpio.pulse(gpio_on=1<<gpio, gpio_off=0, delay=int(width_ms*1000))
    dn = pigpio.pulse(gpio_on=0,       gpio_off=1<<gpio, delay=1)
    pi.wave_add_generic([up, dn])
    return pi.wave_create()

def read_one_int(ser):
    """Read one line and parse int; return None on empty/invalid."""
    s = ser.readline().decode(errors="ignore").strip()
    if not s: return None
    try:
        return int(s)
    except ValueError:
        return None

def main():
    ap = argparse.ArgumentParser(description="PBT amplitude → pulse with live plot (throttled).")
    ap.add_argument("--port", default=SERIAL_PORT)
    ap.add_argument("--baud", type=int, default=BAUD)
    ap.add_argument("--gpio", type=int, default=GPIO_PULSE)
    ap.add_argument("--print-peaks", action="store_true")
    args = ap.parse_args()

    # pigpio
    pi = pigpio.pi()
    if not pi.connected:
        print("ERROR: pigpio daemon not running. Run: sudo systemctl start pigpiod", file=sys.stderr)
        sys.exit(1)
    pi.set_mode(args.gpio, pigpio.OUTPUT)
    pi.write(args.gpio, 0)

    # serial
    ser = serial.Serial(args.port, args.baud, timeout=1)
    time.sleep(0.2); ser.reset_input_buffer()

    # quick baseline warm-up (200 ms)
    baseline, env = 0.0, 0.0
    armed, peak, cap_end = True, 0.0, 0.0

    t0 = time.time()
    n = 0
    while time.time() - t0 < 0.2:
        v = read_one_int(ser)
        if v is None: continue
        baseline = (baseline*n + v) / (n+1) if n > 0 else v
        n += 1
    print(f"Baseline ≈ {baseline:.1f} (0–1023 ADC counts)")

    # plot setup
    raw_buf = deque([baseline]*N_PLOT, maxlen=N_PLOT)
    env_buf = deque([0]*N_PLOT,        maxlen=N_PLOT)
    plt.ion()
    fig, ax = plt.subplots()
    x = range(N_PLOT)
    line_raw, = ax.plot(x, list(raw_buf), label="Raw")
    line_env, = ax.plot(x, list(env_buf), label="Envelope")
    ax.axhline(y=TRIGGER_THRESHOLD, linestyle="--", label="Threshold")
    ax.set_xlim(0, N_PLOT-1)
    ax.set_ylim(0, 1023)
    ax.set_title("PBT Raw vs Envelope")
    ax.set_ylabel("ADC counts")
    ax.legend()

    # stats
    samp_count, t_last = 0, time.time()

    # re-arm level to avoid re-trigger while env is still high
    REARM_LEVEL = TRIGGER_THRESHOLD * 0.4

    try:
        plot_tick = 0
        while True:
            # 1) Drain backlog quickly (so hits don't stretch over seconds)
            consumed = 0
            while ser.in_waiting > 0 and consumed < 2000:
                v = read_one_int(ser)
                if v is None: break
                consumed += 1
                samp_count += 1

                # update baseline/env
                baseline = (1 - BASELINE_ALPHA)*baseline + BASELINE_ALPHA*v
                xmag = abs(v - baseline)
                env   = (1 - ENVELOPE_ALPHA)*env     + ENVELOPE_ALPHA*xmag

                now = time.time()

                if armed:
                    if env > TRIGGER_THRESHOLD:
                        armed  = False
                        peak   = env
                        cap_end = now + (CAPTURE_MS/1000.0)
                else:
                    if env > peak:
                        peak = env
                    if now >= cap_end or env < (TRIGGER_THRESHOLD * 0.5):
                        # map & fire pulse
                        a_clamped = clamp(peak, A_MIN, A_MAX)
                        width_ms  = clamp(map_linear(a_clamped, A_MIN, A_MAX, W_MIN_MS, W_MAX_MS),
                                          W_MIN_MS, W_MAX_MS)
                        if args.print_peaks:
                            print(f"Peak={peak:.1f} → {width_ms:.0f} ms")

                        wid = build_pulse_wave(pi, args.gpio, width_ms)
                        if wid >= 0:
                            pi.wave_send_once(wid)
                            while pi.wave_tx_busy():
                                time.sleep(0.001)
                            pi.wave_delete(wid)
                        else:
                            pi.write(args.gpio, 1); time.sleep(width_ms/1000.0); pi.write(args.gpio, 0)

                        # refractory, but do NOT re-arm until env is truly low
                        t_ref_end = time.time() + (REFRACTORY_MS/1000.0)
                        while time.time() < t_ref_end:
                            v2 = read_one_int(ser)
                            if v2 is None: continue
                            samp_count += 1
                            baseline = (1-BASELINE_ALPHA)*baseline + BASELINE_ALPHA*v2
                            xmag = abs(v2 - baseline)
                            env  = (1-ENVELOPE_ALPHA)*env + ENVELOPE_ALPHA*xmag

                        # wait to re-arm until env falls below REARM_LEVEL
                        while True:
                            if env < REARM_LEVEL:
                                armed = True
                                break
                            v3 = read_one_int(ser)
                            if v3 is None:
                                time.sleep(0.001);  # yield briefly
                                continue
                            samp_count += 1
                            baseline = (1-BASELINE_ALPHA)*baseline + BASELINE_ALPHA*v3
                            xmag = abs(v3 - baseline)
                            env  = (1-ENVELOPE_ALPHA)*env + ENVELOPE_ALPHA*xmag

                # update plot buffers for each consumed sample
                raw_buf.append(v)
                env_buf.append(env)

                # throttle UI updates
                plot_tick += 1
                if plot_tick % PLOT_EVERY == 0:
                    line_raw.set_ydata(raw_buf)
                    line_env.set_ydata(env_buf)
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()

            # fallback blocking read if no backlog
            if ser.in_waiting == 0:
                v = read_one_int(ser)
                if v is not None:
                    samp_count += 1
                    baseline = (1-BASELINE_ALPHA)*baseline + BASELINE_ALPHA*v
                    xmag = abs(v - baseline)
                    env  = (1-ENVELOPE_ALPHA)*env + ENVELOPE_ALPHA*xmag
                    raw_buf.append(v); env_buf.append(env)
                    plot_tick += 1
                    if plot_tick % PLOT_EVERY == 0:
                        line_raw.set_ydata(raw_buf)
                        line_env.set_ydata(env_buf)
                        fig.canvas.draw_idle()
                        fig.canvas.flush_events()

            # print effective SPS once per second (for sanity)
            if time.time() - t_last >= 1.0:
                print(f"SPS: {samp_count}")
                samp_count = 0
                t_last = time.time()

    except KeyboardInterrupt:
        pass
    finally:
        pi.write(args.gpio, 0)
        pi.stop()
        ser.close()

if __name__ == "__main__":
    main()
