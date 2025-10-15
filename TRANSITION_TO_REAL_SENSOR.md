# Transition from Simulator to Real PBT Sensor

## 🎯 Quick Summary

**Current State**: Using fake signal generator (`signal_simulator.ino`)  
**Goal**: Replace with real piezoelectric sensor  
**Impact on Pi software**: **ZERO** - it's all Arduino-side! ✅

---

## 📁 Files Created for You

### Documentation
1. **`REAL_PBT_IMPLEMENTATION_GUIDE.md`** - Complete hardware guide
2. **`ARDUINO_CODE_COMPARISON.md`** - Which Arduino code to use
3. **`CIRCUIT_DIAGRAMS.txt`** - Circuit schematics (ASCII art)
4. **`TRANSITION_TO_REAL_SENSOR.md`** - This file

### Arduino Code (Ready to Upload)
1. **`pbt_sensor_basic.ino`** - Simplest version for testing
2. **`pbt_sensor_averaged.ino`** - With noise filtering
3. **`pbt_sensor_calibrated.ino`** - Auto-baseline correction
4. **`pbt_sensor_full.ino`** - ⭐ Full-featured (recommended)

All located in: `SICK-Capstone/arduino_stuff/`

---

## 🚀 Quick Start Path

### Phase 1: Get Hardware (Budget: $5-15)

**Minimum (for testing)**:
```
□ Piezoelectric sensor ($2-5)
□ 1MΩ resistor ($0.10)
□ Breadboard + wires ($5)
```

**Recommended (for production)**:
```
□ Above parts
□ LM358 op-amp ($0.30)
□ Resistors: 4×10kΩ, 1×100kΩ ($1)
□ Capacitors: 10µF, 100nF ($2)
```

**Where**: Amazon, SparkFun, Adafruit, DigiKey, local electronics store

### Phase 2: Build Simple Circuit (5 minutes)

```
PBT Sensor (+) ──── 1MΩ resistor ──┬──── Arduino A0
                                   │
PBT Sensor (-) ────────────────────┴──── Arduino GND
```

That's it! Start with this simple circuit.

### Phase 3: Upload Arduino Code (2 minutes)

1. Open Arduino IDE
2. File → Open → `arduino_stuff/pbt_sensor_basic/pbt_sensor_basic.ino`
3. Tools → Board → Select your Arduino model
4. Tools → Port → Select correct port
5. Upload (→ button)

### Phase 4: Test Locally (1 minute)

1. Tools → Serial Monitor
2. Set baud rate: **115200**
3. Should see numbers streaming: `512 513 511 ...`
4. Tap the sensor → numbers should spike!

### Phase 5: Connect to Pi (Already works!)

1. Connect Arduino to Raspberry Pi via USB
2. Run: `./start_combined.sh`
3. Open web interface: `http://<pi-ip>:5000`
4. Tap sensor → see spikes in web UI
5. Should see pulse count incrementing
6. GPIO pin 18 should output pulses

**Done!** 🎉

---

## 📊 Decision Matrix

### "I just want to see if it works"
**Hardware**: Simple circuit (PBT + 1MΩ resistor)  
**Code**: `pbt_sensor_basic.ino`  
**Time**: 10 minutes  
**Cost**: $5

### "I want decent performance"
**Hardware**: Simple circuit  
**Code**: `pbt_sensor_full.ino` (has noise filtering + calibration)  
**Time**: 10 minutes  
**Cost**: $5

### "I want production-ready system"
**Hardware**: Circuit with amplifier (see CIRCUIT_DIAGRAMS.txt)  
**Code**: `pbt_sensor_full.ino`  
**Time**: 30 minutes  
**Cost**: $15

---

## 🔧 What You'll Need to Tune

After getting basic hardware working, you may need to adjust:

### 1. Hardware Gain
If signal is:
- **Too weak**: Add amplifier, increase gain
- **Too strong**: Reduce gain, use voltage divider
- **Just right**: Perfect! ✅

### 2. Software Threshold
Edit on Pi: `/SICK-App/config.py`
```python
TRIGGER_THRESHOLD = 60  # Lower = more sensitive
```

### 3. Envelope Settings
```python
ENVELOPE_ALPHA = 0.12      # Lower = smoother (0.01-0.3)
BASELINE_ALPHA = 0.001     # Lower = more stable (0.0001-0.01)
```

---

## 🧪 Expected Signal Characteristics

### Good Signal Looks Like:
```
No impact:   Baseline ~512, small noise ±10
Light tap:   Peak at 600-700 (100-200 above baseline)
Hard impact: Peak at 800-900 (300-400 above baseline)
Max impact:  Peak at 900-1000 (not clipping at 1023)
```

### Problem Signals:

**Too Flat** (barely changes):
```
Cause: Sensor output too weak
Fix: Add amplifier, check connections
```

**Too Spiky** (hits 0 or 1023):
```
Cause: Too much gain
Fix: Reduce amplifier gain, add attenuation
```

**Too Noisy** (jumps around constantly):
```
Cause: Electrical noise, poor connections
Fix: Use shielded cable, add filtering, use averaged code
```

---

## 📈 Progressive Enhancement Path

### Level 1: Proof of Concept ✅
- Simple circuit
- Basic Arduino code
- Verify sensor responds to impacts
- **Success criteria**: See spikes when tapping

### Level 2: Working System ✅✅
- Simple circuit or basic amplifier
- Full-featured Arduino code
- Tune threshold on Pi
- **Success criteria**: Reliable detection, appropriate GPIO pulses

### Level 3: Optimized System ✅✅✅
- Custom amplifier with tuned gain
- Calibrated Arduino code
- Optimized parameters
- **Success criteria**: Consistent performance, minimal false triggers

### Level 4: Production System ✅✅✅✅
- PCB or permanent soldered circuit
- Enclosure for electronics
- Documented calibration procedure
- **Success criteria**: Repeatable, reliable, maintainable

---

## 🐛 Common Issues & Solutions

### Issue: Arduino uploads but no change in signal
```
Check:
□ Correct pin (A0) in both code and wiring
□ Sensor actually connected to breadboard
□ Breadboard connections solid (try re-inserting)
□ Test with multimeter (should read ~2.5V at A0)
```

### Issue: Signal responds to taps but web interface doesn't show spikes
```
Check:
□ Pi is receiving data: sudo cat /dev/ttyUSB0 (should see numbers)
□ Correct serial port in config.py
□ App is running: ps aux | grep app_combined
□ Browser cache - try hard refresh (Ctrl+Shift+R)
```

### Issue: Spikes detected but no GPIO pulses
```
Check:
□ pigpiod daemon running: sudo systemctl status pigpiod
□ GPIO pin physically connected (LED test)
□ Check console for "Pulse #X" messages
□ Threshold too high (lower TRIGGER_THRESHOLD)
```

### Issue: Constant false triggers
```
Fix:
□ Increase TRIGGER_THRESHOLD in config.py
□ Improve shielding/grounding
□ Add mechanical damping to sensor
□ Use averaged or calibrated Arduino code
```

---

## 🎓 Learning Resources

### Piezoelectric Sensors
- How they work: Generate voltage when stressed
- Output: AC signal (positive and negative)
- Impedance: Very high (need high-input impedance circuit)

### Op-Amp Basics
- LM358 datasheet: Good beginner op-amp
- Non-inverting amplifier: Increases signal without inversion
- Gain = 1 + (Rf/Rin) where Rf is feedback resistor

### ADC (Analog-to-Digital Converter)
- Arduino Uno: 10-bit (0-1023)
- Reference: 5V (or 3.3V for 3.3V boards)
- Resolution: 5V / 1024 = 4.88 mV per step

---

## ✅ Validation Checklist

Before considering it "done":

**Hardware**:
- [ ] Sensor mounted securely
- [ ] Wiring is solid (no loose connections)
- [ ] Circuit tested independently
- [ ] Signal range verified (not clipping)

**Arduino**:
- [ ] Code uploaded successfully
- [ ] Serial Monitor shows streaming data
- [ ] Taps produce visible spikes
- [ ] Sample rate ~800 Hz verified

**Raspberry Pi**:
- [ ] Serial port accessible
- [ ] pigpiod running
- [ ] App starts without errors
- [ ] Web interface loads

**System Integration**:
- [ ] Web shows real-time waveforms
- [ ] Taps trigger peak detection
- [ ] GPIO pulses generated
- [ ] Pulse count increments
- [ ] Pulse widths vary with impact strength

**Performance**:
- [ ] Detects all intended impacts
- [ ] Minimal false triggers
- [ ] Appropriate pulse widths
- [ ] Stable over time

---

## 🎯 Next Steps Recommendation

### Week 1: Hardware Acquisition
- Order parts online or visit electronics store
- Read documentation (REAL_PBT_IMPLEMENTATION_GUIDE.md)
- Gather tools (breadboard, wires, multimeter if available)

### Week 2: Basic Implementation
- Build simple circuit
- Upload pbt_sensor_basic.ino
- Verify with Serial Monitor
- Test with Pi system

### Week 3: Optimization
- Add amplifier if needed
- Upload pbt_sensor_full.ino
- Tune threshold and parameters
- Test with real application

### Week 4: Refinement
- Document optimal settings
- Create permanent circuit (optional)
- Test reliability over time
- Prepare for deployment

---

## 📞 Support

If you get stuck:

1. **Check documentation**:
   - REAL_PBT_IMPLEMENTATION_GUIDE.md (hardware details)
   - ARDUINO_CODE_COMPARISON.md (code selection)
   - CIRCUIT_DIAGRAMS.txt (wiring)

2. **Test systematically**:
   - Verify each component independently
   - Use Serial Monitor to debug Arduino side
   - Check web interface for Pi side

3. **Common gotchas**:
   - Breadboard connections (try re-inserting)
   - Pin numbering (A0 = Analog 0, not Digital 0)
   - Polarity (some components have +/-)
   - Loose wires (jiggle test)

---

## 🎉 Success Looks Like

When everything is working:

**On Arduino Serial Monitor**:
```
512
513
511
[tap sensor]
689
823
741
612
523
512
```

**On Web Interface**:
- Live waveform showing clear peaks on impacts
- Envelope smoothly following signal
- Baseline stable around 40-60
- Pulse count incrementing with each impact

**On Console**:
```
Pulse #1: Peak=234.5 → 520 ms
Pulse #2: Peak=456.8 → 890 ms
Pulse #3: Peak=189.2 → 420 ms
```

**On GPIO Pin 18**:
- Pulse widths proportional to impact strength
- Clean on/off transitions
- No false triggers

---

## 🔮 Future Enhancements

Once basic system works:
- [ ] Multiple sensors (different impact types)
- [ ] Frequency analysis (FFT for material detection)
- [ ] Machine learning (impact classification)
- [ ] Data logging (CSV export)
- [ ] Cloud integration (remote monitoring)
- [ ] Mobile app (control from phone)

---

**You've got this!** The transition from simulator to real sensor is straightforward. Start simple, test incrementally, and don't hesitate to iterate. 💪

Good luck! 🚀

