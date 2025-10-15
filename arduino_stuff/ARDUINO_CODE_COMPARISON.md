# Arduino PBT Sensor Code Comparison

## Available Sketches

### 1. `signal_simulator.ino` (Current - FAKE)
**Location**: `arduino_stuff/signal_simulator/`
**Purpose**: Generate fake PBT signals for testing
**Status**: ✅ Currently in use

**Features**:
- Simulated baseline with noise
- Random "hits" every 2-3 seconds
- Half-sine envelope
- No real sensor needed

**Use when**: Testing software without hardware

---

### 2. `pbt_sensor_basic.ino` (NEW - REAL)
**Location**: `arduino_stuff/pbt_sensor_basic/`
**Purpose**: Simplest real sensor implementation
**Recommended for**: First hardware test

**Features**:
- ✅ Direct ADC read from A0
- ✅ 800 Hz sample rate
- ✅ No calibration
- ✅ Minimal code (~30 lines)

**Circuit Required**:
```
PBT → 1MΩ resistor → Arduino A0
      ↓
     GND
```

**Pros**: 
- Simple to understand
- Easy to debug
- Quick to test

**Cons**: 
- No noise filtering
- No baseline correction
- May need external amplifier

---

### 3. `pbt_sensor_averaged.ino` (NEW - REAL)
**Location**: `arduino_stuff/pbt_sensor_averaged/`
**Purpose**: Real sensor with noise reduction
**Recommended for**: Noisy environments

**Features**:
- ✅ Averages 4 ADC readings per sample
- ✅ Reduces electrical noise
- ✅ Still runs at 800 Hz output
- ✅ Simple code (~35 lines)

**Circuit Required**: Same as basic version

**Pros**: 
- Cleaner signal
- Better for low-cost sensors
- Minimal performance impact

**Cons**: 
- Slightly more CPU time
- Still no calibration

---

### 4. `pbt_sensor_calibrated.ino` (NEW - REAL)
**Location**: `arduino_stuff/pbt_sensor_calibrated/`
**Purpose**: Real sensor with auto-calibration
**Recommended for**: Production use (simple version)

**Features**:
- ✅ Auto-finds baseline on startup
- ✅ Centers signal around 512
- ✅ Handles DC offset issues
- ✅ 1.25 second calibration period
- ✅ Status messages to serial

**Circuit Required**: Same as basic, but benefits from amplifier

**Pros**: 
- No manual tuning needed
- Handles baseline drift
- Good for varying conditions

**Cons**: 
- Must be stationary during startup
- 1.25s delay before data starts

**Startup Output**:
```
# CALIBRATING...
# BASELINE=523
# READY
512
513
...
```

---

### 5. `pbt_sensor_full.ino` (NEW - REAL) ⭐ Recommended
**Location**: `arduino_stuff/pbt_sensor_full/`
**Purpose**: Full-featured real sensor
**Recommended for**: Final deployment

**Features**:
- ✅ Auto-calibration
- ✅ Noise averaging
- ✅ LED activity indicator
- ✅ Diagnostic statistics every 5s
- ✅ Min/max tracking
- ✅ Visual feedback

**Circuit Required**: Same as others + LED (optional, uses built-in LED)

**Pros**: 
- Best of all features
- Easy to troubleshoot
- Visual confirmation of activity
- Built-in diagnostics

**Cons**: 
- Most complex code
- Slightly more CPU usage

**Diagnostic Output**:
```
# PBT Sensor Full Version
# CALIBRATING...
# BASELINE=517
# READY
512
515
...
# STATS: samples=4000 min=480 max=720 range=240
```

The STATS line tells you:
- How many samples sent
- Signal min/max values
- Dynamic range

Use this to tune your amplifier gain!

---

## Quick Decision Guide

### "I just want to test if my sensor works"
→ Use `pbt_sensor_basic.ino`

### "My signal is noisy"
→ Use `pbt_sensor_averaged.ino`

### "My baseline keeps drifting"
→ Use `pbt_sensor_calibrated.ino`

### "I want the best, production-ready version"
→ Use `pbt_sensor_full.ino` ⭐

### "I don't have hardware yet"
→ Keep using `signal_simulator.ino`

---

## Migration Path

### Step 1: Get Hardware
- PBT sensor (piezo disc or contact mic)
- 1MΩ resistor
- Breadboard and wires
- (Optional) Op-amp for amplification

### Step 2: Build Simple Circuit
```
PBT(+) ──┤├── 1MΩ ──┬── Arduino A0
        10µF        │
PBT(-) ────────────┴── GND
```

### Step 3: Upload Basic Code
1. Open Arduino IDE
2. File → Open → `pbt_sensor_basic.ino`
3. Upload to Arduino
4. Tools → Serial Monitor (115200 baud)
5. Should see numbers streaming

### Step 4: Test with Pi
1. Keep Arduino connected to Pi
2. Run your combined app: `./start_combined.sh`
3. Open web interface
4. Tap the sensor - should see spikes!

### Step 5: Upgrade Code
- If signal is clean: Done! ✅
- If signal is noisy: Upload `pbt_sensor_averaged.ino`
- If baseline drifts: Upload `pbt_sensor_calibrated.ino`
- For final version: Upload `pbt_sensor_full.ino`

### Step 6: Tune System
1. Adjust `TRIGGER_THRESHOLD` in Pi's `config.py`
2. Adjust amplifier gain (if using amp)
3. Test with real impacts
4. Document working settings

---

## Code Compatibility

All new sketches are **100% compatible** with your existing Pi software:
- ✅ Same 800 Hz sample rate
- ✅ Same serial format (integers 0-1023)
- ✅ Same baud rate (115200)
- ✅ Drop-in replacement for `signal_simulator.ino`

**No changes needed** to `app_combined.py` or web interface!

---

## Troubleshooting

### Arduino uploads but no data on Pi
- Check serial port: `ls /dev/ttyUSB*`
- Verify baud rate: Should be 115200
- Reset Arduino after connecting to Pi

### Signal is always 0 or always 1023
- Check sensor connections
- Verify A0 pin is connected
- Try reading with Arduino Serial Monitor first

### Signal doesn't change when tapping sensor
- Check sensor polarity (+ and - connections)
- Verify sensor is working (test with multimeter)
- May need amplifier for weak signal

### LED doesn't blink (full version)
- Check if impacts are strong enough
- Lower `activityThreshold` in code (default 50)
- Test with manual taps directly on sensor

### Baseline is not near 512
- Use calibrated or full version for auto-correction
- Or add DC offset in hardware (2.5V)
- Or adjust in Python code

---

## Performance Comparison

| Feature | Simulator | Basic | Averaged | Calibrated | Full |
|---------|-----------|-------|----------|------------|------|
| Real Sensor | ❌ | ✅ | ✅ | ✅ | ✅ |
| Noise Filter | N/A | ❌ | ✅ | ❌ | ✅ |
| Auto-Calibrate | N/A | ❌ | ❌ | ✅ | ✅ |
| Diagnostics | ❌ | ❌ | ❌ | Minimal | ✅ |
| LED Feedback | ❌ | ❌ | ❌ | ❌ | ✅ |
| Code Lines | ~70 | ~30 | ~35 | ~60 | ~120 |
| CPU Usage | Low | Very Low | Low | Low | Low |
| Memory | 1KB | <1KB | <1KB | <1KB | 1KB |

All versions run efficiently on Arduino Uno, Nano, Mega, etc.

---

## Next Steps

1. **Choose your Arduino code** based on needs
2. **Build the circuit** (start simple!)
3. **Upload the code** to Arduino
4. **Test with Serial Monitor** (verify data streaming)
5. **Connect to Pi** and run combined app
6. **Tune parameters** for your application
7. **Celebrate!** 🎉

Questions? Check `REAL_PBT_IMPLEMENTATION_GUIDE.md` for detailed hardware info.

