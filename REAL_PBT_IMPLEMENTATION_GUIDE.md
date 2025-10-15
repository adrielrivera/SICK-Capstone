# Real PBT Sensor Implementation Guide

## Current State: Simulator

Your current Arduino code (`signal_simulator.ino`) generates **fake** PBT signals:
- Baseline ~40 with noise (±6)
- Random "hits" every 2-3 seconds
- Half-sine envelope (200-900 peak, 120-320ms duration)
- No actual sensor connected

## Goal: Real PBT Sensor

Replace the simulator with actual piezoelectric sensor readings.

---

## Hardware Requirements

### 1. PBT Sensor (Piezoelectric)
- **Type**: Piezoelectric contact microphone or PBT sensor
- **Output**: AC voltage (typically mV to V range)
- **Examples**:
  - Contact microphone element
  - Piezo disc sensor
  - Purpose-built PBT sensor

### 2. Signal Conditioning Circuit
You'll likely need:

#### Option A: Simple Circuit (Quick Test)
```
PBT Sensor → 1MΩ resistor → Arduino A0
              ↓
             GND
```
- Good for: Initial testing
- Limitation: Low sensitivity, no amplification

#### Option B: Op-Amp Amplifier (Better)
```
PBT Sensor → AC Coupling → Amplifier → Offset → Arduino A0
```

Components needed:
- **Op-amp**: LM358, TL072, or similar
- **Gain**: 10x to 100x (adjust based on sensor output)
- **AC coupling**: Remove DC component (10µF capacitor + resistor)
- **Offset**: Add 2.5V DC to center signal in 0-5V range
- **Filtering**: Optional low-pass filter to reduce noise

#### Option C: Off-the-shelf Module
- Piezo sensor breakout board with built-in amplifier
- Examples: Grove Piezo Vibration Sensor, SparkFun Piezo modules
- Easiest option if available

### 3. Arduino Board
- **Current**: Already using one (connected as /dev/ttyUSB0)
- **Requirements**: At least 1 analog input pin
- **ADC**: 10-bit (0-1023) - already compatible with your code

---

## Circuit Design Considerations

### Signal Chain

```
Physical Impact → PBT Sensor → Signal Conditioning → Arduino ADC → Serial → Pi
```

### Key Issues to Address

#### 1. **Signal Polarity**
- Piezo sensors produce **bipolar AC signals** (both positive and negative)
- Arduino ADC reads **0-5V only** (or 0-3.3V for some boards)
- **Solution**: Add DC offset to center signal around 2.5V

#### 2. **Signal Amplitude**
- Raw piezo output might be too weak (mV range)
- Or too strong (several volts on hard impact)
- **Solution**: Add amplifier with adjustable gain

#### 3. **Baseline Drift**
- DC coupling causes baseline drift
- **Solution**: AC coupling with capacitor (removes DC, passes AC)

#### 4. **Noise**
- Piezo sensors pick up everything: electrical noise, vibrations, etc.
- **Solution**: Shielding, filtering, good grounding

---

## Recommended Circuit (Step-by-Step)

### Circuit Diagram

```
                    R1 1MΩ
                     │
PBT ───┤├── R2 ───┬──┤
Sensor  C1     10kΩ│  │
               ↓   │  ↓GND
               │   │
            To Op-Amp (optional)
               │
          Offset to 2.5V
               │
          To Arduino A0
```

### Simple Version (No Op-Amp)

```
PBT Sensor (+) ───┤├─── 1MΩ ──┬─── Arduino A0
                   10µF        │
PBT Sensor (-) ─────────────┬──┴─── Arduino GND
                            GND
```

**Pros**: Simple, quick to test
**Cons**: Low sensitivity, may miss weak impacts

### Amplified Version (Recommended)

Use an LM358 or similar op-amp:

```
             C1 10µF    R1 10kΩ
PBT (+) ──┬──┤├────┬────┬────┐
          │         │    │    │
          │        GND  2.5V  │  LM358
          │                   ├──┐ +In
PBT (-) ──┴───────────────────┘  │ -In ──┬── R2 10kΩ
                                  │        │
                          GND ────┤ GND    │
                                  │ Vcc ── 5V
                                  │ Out ───┴──┬── Arduino A0
                                  └───────────┘
```

**Gain Calculation**: `Gain = 1 + (R2/R1)`
- Example: 10kΩ/10kΩ = 2x gain
- Increase R2 for more gain (e.g., 100kΩ → 11x gain)

### 2.5V Offset Generator

```
5V ── 10kΩ ──┬── 2.5V out (to op-amp)
             │
    GND ── 10kΩ
```

Or use Arduino DAC pin if available.

---

## Arduino Code: Real PBT Version

### Version 1: Direct Analog Read (Simplest)

Create `pbt_sensor_v1.ino`:

```cpp
// Real PBT Sensor - Version 1 (Direct ADC Read)
// Replace signal_simulator.ino with this

const int PBT_PIN = A0;           // Analog pin connected to PBT sensor
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;

unsigned long nextSample = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PBT_PIN, INPUT);
  analogReference(DEFAULT);  // Use 5V reference (or INTERNAL for 3.3V boards)
  nextSample = micros();
}

void loop() {
  unsigned long now = micros();
  if ((long)(now - nextSample) >= 0) {
    nextSample += SAMPLE_US;
    
    // Read analog value from PBT sensor
    int value = analogRead(PBT_PIN);
    
    // Send to serial (0-1023)
    Serial.println(value);
  }
}
```

**That's it!** This will read real sensor data at 800 Hz.

### Version 2: With Averaging (Noise Reduction)

```cpp
// Real PBT Sensor - Version 2 (With Averaging)

const int PBT_PIN = A0;
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;
const int AVG_SAMPLES = 4;  // Average 4 readings per output

unsigned long nextSample = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PBT_PIN, INPUT);
  nextSample = micros();
}

void loop() {
  unsigned long now = micros();
  if ((long)(now - nextSample) >= 0) {
    nextSample += SAMPLE_US;
    
    // Average multiple readings to reduce noise
    long sum = 0;
    for (int i = 0; i < AVG_SAMPLES; i++) {
      sum += analogRead(PBT_PIN);
    }
    int value = sum / AVG_SAMPLES;
    
    Serial.println(value);
  }
}
```

### Version 3: With Auto-Calibration

```cpp
// Real PBT Sensor - Version 3 (With Auto-Calibration)

const int PBT_PIN = A0;
const int SAMPLES_PER_SEC = 800;
const unsigned long SAMPLE_US = 1000000UL / SAMPLES_PER_SEC;

unsigned long nextSample = 0;
int baselineOffset = 512;  // Start at midpoint
bool calibrated = false;
int calCount = 0;
long calSum = 0;

void setup() {
  Serial.begin(115200);
  pinMode(PBT_PIN, INPUT);
  nextSample = micros();
}

void loop() {
  unsigned long now = micros();
  if ((long)(now - nextSample) >= 0) {
    nextSample += SAMPLE_US;
    
    int rawValue = analogRead(PBT_PIN);
    
    // Auto-calibrate baseline for first 1000 samples (1.25 seconds at 800 Hz)
    if (!calibrated) {
      calSum += rawValue;
      calCount++;
      if (calCount >= 1000) {
        baselineOffset = calSum / calCount;
        calibrated = true;
        // Send a marker to indicate calibration done (optional)
        Serial.println(baselineOffset);
      }
      return;  // Don't send data during calibration
    }
    
    // Send calibrated value
    int value = rawValue - baselineOffset + 512;  // Re-center around 512
    
    // Clamp to 0-1023
    if (value < 0) value = 0;
    if (value > 1023) value = 1023;
    
    Serial.println(value);
  }
}
```

---

## Testing Strategy

### Phase 1: Baseline Test (No Impacts)
1. Upload simple version (v1)
2. Don't touch sensor
3. Check web interface - should see stable baseline around 512
4. Verify sample rate (800 Hz)

**Expected**: Flat line with small noise

### Phase 2: Manual Impact Test
1. Tap the sensor gently
2. Observe spike in web interface
3. Check if envelope detector triggers
4. Verify GPIO pulse is generated

**Expected**: Clear peaks on taps, pulse output

### Phase 3: Real Application Test
1. Mount sensor on actual surface/object
2. Apply real impacts
3. Tune threshold in `config.py`
4. Adjust amplifier gain if needed

**Expected**: Reliable detection, appropriate pulse widths

---

## Calibration & Tuning

### 1. Baseline Adjustment

If baseline is not around 512:
- **Too low (<200)**: Increase DC offset voltage
- **Too high (>800)**: Decrease DC offset voltage
- **Drifting**: Check AC coupling capacitor

### 2. Sensitivity Adjustment

If sensor is:
- **Too sensitive**: 
  - Reduce amplifier gain
  - Increase `TRIGGER_THRESHOLD` in `config.py`
  - Add damping material to sensor
  
- **Not sensitive enough**:
  - Increase amplifier gain
  - Decrease `TRIGGER_THRESHOLD` in `config.py`
  - Improve sensor mounting

### 3. Signal Processing Parameters

Edit `/SICK-App/config.py`:

```python
# Adjust these based on real sensor behavior
TRIGGER_THRESHOLD = 60        # Lower for more sensitive, higher for less
ENVELOPE_ALPHA = 0.12         # Lower for smoother envelope (0.01-0.3)
BASELINE_ALPHA = 0.001        # Lower for more stable baseline (0.0001-0.01)

# Pulse mapping (from pbt_pulse_plot.py logic)
A_MIN, A_MAX = 80, 700        # Min/max amplitude for mapping
W_MIN_MS, W_MAX_MS = 60, 1500 # Min/max pulse width output
```

### 4. Amplifier Gain Tuning

Target signal range:
- **Baseline**: ~512 (centered)
- **Weak impact**: 100-200 counts above baseline
- **Strong impact**: 400-600 counts above baseline
- **Max impact**: Should not clip at 1023

Calculate gain needed:
```
Gain = (Target ADC range) / (Sensor output range)

Example:
- Sensor outputs: 10mV to 500mV
- Arduino ADC: 0-5V (with 2.5V offset)
- Available range: ±2.5V = 2500mV
- Target range: ±500 ADC counts = ~2.5V
- Gain needed: 2500mV / 500mV = 5x
```

---

## Common Issues & Solutions

### Issue 1: Signal Clipping
**Symptom**: Peaks always hit 1023 or 0
**Solution**: 
- Reduce amplifier gain
- Add protective diode clamps
- Reduce impact force for testing

### Issue 2: Too Much Noise
**Symptom**: Baseline jumps around ±50 or more
**Solution**:
- Add decoupling capacitors (100nF near Arduino)
- Use shielded cable for sensor
- Improve grounding
- Add low-pass filter (RC filter, cutoff ~500 Hz)
- Move sensor away from EMI sources

### Issue 3: No Signal at All
**Symptom**: Flat line at 0, 512, or 1023
**Solution**:
- Check sensor connections
- Verify sensor is working (tap while measuring with multimeter)
- Check Arduino pin in code matches hardware
- Verify power supply to amplifier

### Issue 4: Baseline Drift
**Symptom**: Baseline slowly rises or falls
**Solution**:
- Increase AC coupling capacitor (use 100µF instead of 10µF)
- Check for temperature effects
- Ensure good DC offset stability

### Issue 5: False Triggers
**Symptom**: Pulses generated without real impacts
**Solution**:
- Increase `TRIGGER_THRESHOLD`
- Dampen sensor mechanically
- Shield from vibrations
- Adjust `ENVELOPE_ALPHA` for slower response

---

## Recommended Testing Equipment

1. **Multimeter**: Check voltages at each stage
2. **Oscilloscope** (if available): View raw sensor signal, amplifier output
3. **LED + resistor**: Connect to GPIO 18 to visually confirm pulses
4. **Spare Arduino**: Keep simulator running for comparison

---

## Migration Checklist

- [ ] Choose circuit design (simple vs. amplified)
- [ ] Gather components (PBT sensor, resistors, caps, op-amp)
- [ ] Build circuit on breadboard
- [ ] Test sensor output with multimeter (tap sensor)
- [ ] Upload Arduino code v1 (direct read)
- [ ] Connect to Pi and run web app
- [ ] Verify baseline reading (~512)
- [ ] Test manual taps - check for peaks
- [ ] Tune threshold and sensitivity
- [ ] Test GPIO pulse output
- [ ] Build permanent circuit (optional)
- [ ] Mount sensor on target surface
- [ ] Final calibration and testing

---

## Next Steps

### Immediate (Testing Phase)
1. **Get a PBT sensor** - Order piezo disc or contact microphone
2. **Build simple circuit** - Just sensor → 1MΩ → A0 for initial test
3. **Upload v1 code** - Direct analog read
4. **Run combined app** - See if you get any signal

### Short Term (Development)
1. **Add amplification** - Build op-amp circuit for better sensitivity
2. **Tune parameters** - Adjust threshold, gain, envelope settings
3. **Test real use case** - Mount on actual target
4. **Document optimal settings** - Record working configuration

### Long Term (Production)
1. **Design PCB** - Make permanent circuit board
2. **Enclosure** - 3D print case for electronics
3. **Calibration procedure** - Document setup process
4. **User manual** - How to install and configure

---

## Code Files Needed

I can create these Arduino sketches for you:

1. `pbt_sensor_basic.ino` - Simple direct read
2. `pbt_sensor_averaged.ino` - With noise reduction
3. `pbt_sensor_calibrated.ino` - Auto-baseline calibration
4. `pbt_sensor_full.ino` - All features + diagnostics

Would you like me to create these files?

---

## References & Resources

### Piezoelectric Sensors
- Contact microphone elements (~$1-5 each)
- Piezo disc sensors (20-35mm diameter)
- Industrial PBT sensors (higher quality, more $$$)

### Op-Amp Circuits
- LM358 datasheet (dual op-amp, cheap, easy to use)
- TL072 (lower noise, better for audio)
- Instrumentation amplifier (INA128) for best performance

### Arduino ADC
- 10-bit resolution (0-1023)
- Default reference: 5V (or 3.3V for 3.3V boards)
- Can use external AREF for custom reference voltage

---

## Questions to Consider

1. **What type of impact are you detecting?**
   - Light taps vs. heavy hits
   - Frequency content (sharp vs. sustained)
   - Material (metal, wood, plastic)

2. **Where will the sensor be mounted?**
   - Direct contact with impact surface
   - Remote mounting (detect vibration transmission)

3. **What's your budget?**
   - DIY piezo disc: $1-5
   - Breakout board with amp: $5-15
   - Industrial sensor: $50-200+

4. **Do you have test equipment?**
   - Oscilloscope helpful but not required
   - Multimeter sufficient for basic testing

Let me know what hardware you have or plan to get, and I can provide more specific guidance!

