# SICK 10 Bar PBT Sensor - Complete Setup Guide

Based on your oscilloscope measurements! 📊

---

## 🎯 Your Sensor Characteristics

From the oscilloscope trace you shared:

| Parameter | Value | Arduino ADC |
|-----------|-------|-------------|
| **Baseline** | 1.88V | ~385 counts |
| **Peak** | 2.96V | ~607 counts |
| **Amplitude** | 1.1V | ~222 counts |
| **Pulse Duration** | ~710ms | Long decay |
| **Shape** | Sharp rise + exponential decay | Classic impact |

**Good news**: This signal is **perfect for direct Arduino connection** - no amplifier needed! ✅

---

## 🔌 Hardware Setup

### Wiring (Super Simple!)

```
SICK 10 Bar PBT Sensor
┌──────────────────┐
│                  │
│  Signal Output   ├──── 100kΩ resistor ───┬──── Arduino A0
│                  │                        │
│  Ground          ├────────────────────────┴──── Arduino GND
│                  │
│  (Power if any)  ├──── (Check sensor datasheet)
└──────────────────┘
```

**Components needed**:
- ✅ SICK 10 Bar PBT sensor (you have this!)
- ✅ 100kΩ resistor (brown-black-yellow color code)
- ✅ 2 jumper wires
- ✅ Arduino Uno/Nano/Mega (already connected)

**That's it!** No amplifier, no complex circuit.

### Why 100kΩ?
The SICK sensor has high output impedance. The 100kΩ resistor provides:
- Input impedance matching
- ESD protection
- Slight filtering of very high-frequency noise

### Breadboard Layout

```
┌─────────────────────────────────────┐
│  Breadboard                         │
│                                     │
│  Row 1:  SICK Signal ───────────┐   │
│                                 │   │
│  Row 2:         │               │   │
│                 │               │   │
│  Row 3:         ├── 100kΩ ──────┤   │
│                 │               │   │
│  Row 4:         │               │   │
│                 │               │   │
│  Row 5:         └───────────────┴─► Arduino A0
│                                     │
│  GND:   SICK Ground ───────────────► Arduino GND
│                                     │
└─────────────────────────────────────┘
```

---

## 💻 Arduino Code

### Upload This Code

I've created optimized code specifically for your SICK 10 bar sensor:

**Location**: `arduino_stuff/sick_10bar_sensor/sick_10bar_sensor.ino`

**Features**:
- ✅ Auto-calibrates to your sensor's 1.88V baseline
- ✅ Re-centers signal to 512 (compatible with Pi software)
- ✅ Noise averaging (cleaner signal)
- ✅ LED blinks on impacts
- ✅ Diagnostic output every 5 seconds
- ✅ Sanity checks for proper connection

### Upload Steps

1. Open Arduino IDE
2. File → Open → `sick_10bar_sensor.ino`
3. Tools → Board → Select your Arduino
4. Tools → Port → Select USB port
5. Click Upload (→ button)
6. Wait for "Done uploading"

### Verify It Works

1. Tools → Serial Monitor
2. Set baud rate: **115200**
3. You should see:
   ```
   # SICK 10 Bar PBT Sensor
   # Expected baseline: ~385 ADC counts (1.88V)
   # Expected peak: ~607 ADC counts (2.96V)
   # CALIBRATING...
   # BASELINE MEASURED=387 ADC counts (1.89V)
   # Baseline looks good!
   # READY
   512
   513
   511
   ...
   ```
4. Tap the sensor → numbers should spike!
   ```
   512
   513
   [tap!]
   580
   650
   687
   623
   565
   523
   512
   ```

### Diagnostic Output

Every 5 seconds, you'll see:
```
# STATS: samples=4000 raw_min=382 raw_max=601 range=219 (1.07V)
# High activity detected! Good signal.
```

This tells you:
- **raw_min/max**: Actual ADC values from sensor
- **range**: Total signal swing (should be ~200-220 for impacts)
- **Voltage**: Converted to volts for reference
- **Activity**: Interpretation of signal

---

## 🖥️ Raspberry Pi Configuration

### Update Config File

You have two options:

#### Option 1: Use Specialized Config (Recommended)
```bash
cd /path/to/SICK-App

# Backup current config
cp config.py config_original.py

# Copy SICK 10 bar optimized config
cp config_sick10bar.py config.py
```

#### Option 2: Manually Edit config.py

Change these values in your `config.py`:

```python
# Optimized for SICK 10 bar sensor
ENVELOPE_ALPHA = 0.08      # Lower for 700ms pulses (was 0.12)
TRIGGER_THRESHOLD = 40     # Conservative start (was 60)
BASELINE_ALPHA = 0.0005    # More stable (was 0.001)
```

### Why These Values?

**ENVELOPE_ALPHA = 0.08** (lower than default):
- Your SICK pulses are ~700ms long
- Lower alpha = smoother tracking for long pulses
- Default 0.12 was tuned for short simulated hits

**TRIGGER_THRESHOLD = 40** (lower than default):
- SICK sensor range is ~222 counts
- 40 counts above baseline is ~18% of range
- Start conservative, can adjust based on testing

**BASELINE_ALPHA = 0.0005** (lower than default):
- SICK sensor has very stable baseline (1.88V)
- Lower alpha = more stable tracking
- Prevents baseline from chasing slow drifts

---

## 🚀 Running the System

### Start the Combined App

```bash
cd /path/to/SICK-App
./start_combined.sh
```

You should see:
```
Activating virtual environment...
Starting pigpio daemon...
Starting SICK PBT Combined App...
============================================================
SICK PBT Sensor - Combined Web App + GPIO Pulse Output
============================================================
Serial port: /dev/ttyUSB0 @ 115200 baud
Samples per second: 800
GPIO pulse output: Pin 18
Trigger threshold: 40 ADC counts
Web server: http://0.0.0.0:5000
============================================================
GPIO 18 initialized for pulse output
Baseline calibrated: 512.3 ADC counts
```

### Access Web Interface

Open browser: `http://<pi-ip>:5000`

You should see:
- **Baseline**: ~512 (stable)
- **Envelope**: Low (~0-10) when quiet
- **Live waveform**: Flat line when no impacts

### Test It!

1. **Tap the SICK sensor**
2. **Watch web interface**: Should see spike!
3. **Check console**: Should print pulse info
   ```
   Pulse #1: Peak=134.5 → 580 ms
   ```
4. **Check GPIO**: Pin 18 should output pulse (use LED or scope)

---

## 🎚️ Tuning Parameters

### If You're Missing Impacts

**Symptom**: Light taps don't trigger
**Solution**: Lower threshold
```python
# In config.py
TRIGGER_THRESHOLD = 30  # or even 25
```

### If You Get False Triggers

**Symptom**: Pulses when nothing happened
**Solution**: Raise threshold or improve mounting
```python
# In config.py
TRIGGER_THRESHOLD = 50  # or 60
```

### If Envelope is Too Jumpy

**Symptom**: Envelope value bounces around rapidly
**Solution**: Lower envelope alpha (smoother)
```python
# In config.py
ENVELOPE_ALPHA = 0.05  # Even smoother
```

### If Envelope is Too Slow

**Symptom**: Envelope doesn't follow peaks well
**Solution**: Raise envelope alpha (faster response)
```python
# In config.py
ENVELOPE_ALPHA = 0.12  # Faster tracking
```

---

## 📊 Expected Performance

### Signal Ranges (After Arduino Re-centering)

| Condition | Value Range | Above Baseline |
|-----------|-------------|----------------|
| **Quiet** | 510-514 | 0-2 |
| **Light tap** | 560-600 | 50-90 |
| **Medium impact** | 600-680 | 90-170 |
| **Strong impact** | 680-734 | 170-222 |

### GPIO Pulse Widths

With default mapping (A_MIN=80, A_MAX=700):

| Impact Strength | Envelope Peak | Pulse Width |
|-----------------|---------------|-------------|
| **Very weak** | <80 | 60ms (min) |
| **Light** | 100 | ~150ms |
| **Medium** | 200 | ~500ms |
| **Strong** | 300 | ~850ms |
| **Very strong** | >700 | 1500ms (max) |

Your SICK sensor's range (~222 max) maps nicely to 60-900ms pulse widths.

---

## 🔧 Troubleshooting

### Issue: Baseline Not Around 512

**Check Arduino Serial Monitor**:
```
# BASELINE MEASURED=387 ADC counts (1.89V)
```

This is the RAW baseline (correct for SICK sensor).
The Arduino code re-centers it to 512 for Pi compatibility.

**If you see warning**:
```
# WARNING: Baseline outside expected range 350-450
```

Possible causes:
- Sensor not connected
- Wrong wire (connected to wrong Arduino pin)
- Sensor needs power (check SICK datasheet)

### Issue: No Signal on Impacts

**Check**:
1. Sensor firmly connected to surface/mounting
2. Tapping actual sensor, not nearby area
3. Wire connections solid (jiggle test)
4. Correct Arduino pin (A0)

**Test with multimeter**:
- Measure voltage at Arduino A0
- Should read ~1.88V at rest
- Should jump to ~2.5-2.9V on tap

### Issue: Signal Too Weak

Your SICK sensor outputs good voltage already (~1.1V swing).
If still too weak:

1. **Check mechanical coupling**: Sensor must be well-mounted
2. **Impact directly**: Tap the sensor surface, not nearby
3. **Lower threshold**: Try TRIGGER_THRESHOLD = 25

### Issue: Signal Too Strong/Clipping

**Symptom**: Values hit 0 or 1023
**Cause**: Very unlikely with SICK sensor (good range)
**If it happens**:
- Add voltage divider (2× 100kΩ resistors)
- Or increase TRIGGER_THRESHOLD significantly

### Issue: Noisy Baseline

**Symptom**: Baseline jumps around ±20 or more
**Solutions**:
1. Add capacitor: 100nF between A0 and GND (low-pass filter)
2. Use shielded cable for sensor
3. Check grounding (sensor GND = Arduino GND = Pi GND)
4. Move away from motors, power supplies

### Issue: GPIO Not Pulsing

**Check**:
1. pigpiod running: `sudo systemctl status pigpiod`
2. Threshold triggering: Watch console for "Pulse #X" messages
3. If messages appear but no GPIO: Check pin 18 connection
4. Test with LED: LED + 220Ω resistor to GND on pin 18

---

## 🎯 Validation Checklist

Before considering it fully operational:

**Hardware**:
- [ ] SICK sensor connected: Signal → 100kΩ → A0
- [ ] Ground connected: Sensor GND → Arduino GND
- [ ] Power connected (if SICK requires it - check datasheet)
- [ ] Wires solid (no loose connections)

**Arduino**:
- [ ] Code uploaded: `sick_10bar_sensor.ino`
- [ ] Serial Monitor shows calibration
- [ ] Baseline measured ~387 ADC (1.89V)
- [ ] Taps produce spikes in Serial Monitor
- [ ] LED blinks on impacts

**Raspberry Pi**:
- [ ] Config updated: Using `config_sick10bar.py` or manual edits
- [ ] App starts: `./start_combined.sh`
- [ ] Serial port accessible: `/dev/ttyUSB0`
- [ ] pigpiod running
- [ ] Web interface loads

**System Integration**:
- [ ] Web shows baseline ~512
- [ ] Impacts produce spikes in web graph
- [ ] Envelope tracks impacts smoothly
- [ ] Console shows "Pulse #X" messages
- [ ] Pulse count increments
- [ ] GPIO pin 18 outputs pulses (verify with LED/scope)

**Performance**:
- [ ] All intended impacts detected
- [ ] No false triggers on vibrations
- [ ] Pulse widths appropriate (60-1500ms range)
- [ ] System stable over 10+ minutes
- [ ] Baseline doesn't drift

---

## 📈 Performance Tuning Guide

### Optimization Goals

1. **Sensitivity**: Catch all real impacts
2. **Specificity**: Reject false triggers
3. **Consistency**: Stable over time
4. **Responsiveness**: Quick detection

### Tuning Process

#### Step 1: Baseline Stability (10 minutes)
1. Start system
2. Don't touch sensor
3. Watch web interface
4. Baseline should be 512 ± 5

If drifting:
- Check connections
- Lower BASELINE_ALPHA to 0.0002

#### Step 2: Light Impact Detection (5 minutes)
1. Lightly tap sensor (barely perceptible)
2. Should detect: Envelope should rise to 40+
3. If not: Lower TRIGGER_THRESHOLD to 30

#### Step 3: Strong Impact Detection (5 minutes)
1. Firmly tap sensor (normal use case)
2. Should detect: Envelope should rise to 100-200
3. Check pulse width: Should be 400-1000ms
4. If clipping: Should NOT hit 1023

#### Step 4: False Trigger Test (10 minutes)
1. Vibrate table/surface
2. Tap near (but not on) sensor
3. Should NOT trigger
4. If false triggers: Raise TRIGGER_THRESHOLD to 50

#### Step 5: Sustained Testing (30 minutes)
1. Perform realistic impacts every 10-30 seconds
2. Monitor:
   - Detection rate (should be 100%)
   - False trigger rate (should be 0%)
   - Baseline stability
   - GPIO pulse consistency

---

## 🎓 Understanding the Signal Flow

```
SICK Sensor Impact
       ↓
   1.88V → 2.96V (physical voltage)
       ↓
   100kΩ Resistor (impedance matching)
       ↓
   Arduino A0 ADC
       ↓
   387 → 609 ADC counts (10-bit conversion)
       ↓
   Arduino Processing
   - Averaging (noise reduction)
   - Auto-calibration (find baseline)
   - Re-centering (387 → 512 for compatibility)
       ↓
   Serial @ 115200 baud, 800 Hz
   - Sends: 512 → 734 (re-centered values)
       ↓
   Raspberry Pi (app_combined.py)
   - Baseline tracking
   - Envelope detection
   - Peak detection
   - Amplitude mapping
       ↓
   ├─► Web Interface (Chart.js visualization)
   │   - Real-time waveform
   │   - Statistics
   │   - Pulse count
   │
   └─► GPIO Pin 18 (pigpio)
       - Pulse width: 60-1500ms
       - Proportional to impact strength
```

---

## 📊 Comparison: Simulator vs Real SICK Sensor

| Aspect | Simulator | SICK 10 Bar |
|--------|-----------|-------------|
| **Baseline** | ~40 ADC | ~387 ADC (1.88V) |
| **Peak** | 200-900 ADC | ~607 ADC (2.96V) |
| **Range** | Variable | ~220 ADC counts |
| **Duration** | 120-320ms | ~700ms |
| **Shape** | Half-sine | Sharp rise + exp decay |
| **Noise** | ±6 added | Natural/low |
| **Repeatability** | Perfect | Real-world variation |
| **Frequency** | Every 2-3s | On demand |

The SICK sensor is **much cleaner** and **more predictable** than the simulator!

---

## 🎉 Success Metrics

You'll know it's working perfectly when:

✅ **Visual Confirmation**:
- Web interface shows clean spikes on each tap
- Envelope smoothly follows peaks
- Baseline flat and stable
- No spikes when sensor untouched

✅ **Console Output**:
```
Pulse #1: Peak=134.2 → 580 ms
Pulse #2: Peak=89.5 → 380 ms
Pulse #3: Peak=201.3 → 950 ms
```

✅ **GPIO Output**:
- Oscilloscope shows clean pulses on pin 18
- Pulse widths vary with impact strength
- No pulses during quiet periods

✅ **Reliability**:
- Runs for hours without issues
- Consistent detection
- No random false triggers
- Baseline doesn't drift

---

## 🚀 You're Ready!

With the SICK 10 bar sensor, you have professional-grade hardware. The setup is actually **simpler** than DIY piezo discs because:
- ✅ Clean, calibrated output
- ✅ Good voltage range (no amplifier needed)
- ✅ Stable baseline
- ✅ Professional packaging

Just wire it up, upload the Arduino code, and go! 🎯

Questions? Check the main documentation or test systematically using the troubleshooting section above.

