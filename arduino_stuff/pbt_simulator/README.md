# PBT Simulator Arduino Code

## Overview
This Arduino code is a complete PBT (Punch Bag Trainer) simulator that generates realistic waveforms for testing the scoring system. It combines waveform generation, GPIO control, and LiDAR detection in a single Arduino.

## Features
- **PBT Waveform Generation**: Realistic PBT sensor simulation with noise and peaks
- **Custom Peak Generation**: Generate specific peak amplitudes via serial commands
- **GPIO Control**: Controls arcade motherboard pins 5 and 6
- **LiDAR Detection**: Monitors TiM100 (Left) and TiM150 (Right) sensors
- **Oscilloscope Output**: PWM output on Pin 3 for waveform monitoring
- **Serial Communication**: Responds to commands from Pi

## Hardware Connections

### Oscilloscope Monitoring
```
Arduino Pin 3 (PWM) → Oscilloscope Channel 1 (PBT Waveform)
Arduino Pin 6 (Digital) → Oscilloscope Channel 2 (Arcade Start Signal)
Arduino Pin 5 (Digital) → Oscilloscope Channel 3 (Arcade Active Signal)
Arduino GND → Oscilloscope Ground
```

### Arcade Machine Control
```
Arcade Pin 5 ──── Arduino Pin 5 (ACTIVE signal)
Arcade Pin 6 ──── Arduino Pin 6 (START signal)
Arcade GND  ──── Arduino GND
```

### LiDAR Sensors (Optional)
```
TiM100 (Left) → Arduino Pin 8 (Digital input with pull-down resistor)
TiM150 (Right) → Arduino Pin 9 (Digital input with pull-down resistor)
```

## Serial Commands

### GPIO Control
- `PIN5_HIGH` - Set Pin 5 HIGH (5V)
- `PIN5_LOW` - Set Pin 5 LOW (0V)
- `PIN6_HIGH` - Set Pin 6 HIGH (5V)
- `PIN6_LOW` - Set Pin 6 LOW (0V)
- `RESET_GPIO` - Reset pins to default states

### PBT Simulation
- `CUSTOM_PEAK:amplitude` - Generate custom peak (e.g., `CUSTOM_PEAK:45`)
- `START_SIMULATION` - Start waveform generation
- `STOP_SIMULATION` - Stop waveform generation

### Status
- `STATUS` - Report current status
- `PBT_HIT` - Acknowledge PBT hit from Pi

## Waveform Characteristics
- **Baseline**: ~2.5V (512 ADC = 128 PWM)
- **Noise**: ±0.1V random variation
- **Peaks**: 0.5V to 2.5V spikes (30-100 ADC = 8-25 PWM)
- **Frequency**: 800Hz sampling rate
- **Peak Duration**: ~62.5ms (50 samples at 800Hz)

## Upload Instructions

### Using the upload script (Recommended)
```bash
cd ~/SICK-Capstone/SICK-Capstone/arduino_stuff/pbt_simulator
./upload.sh
```

### Using Arduino IDE
1. Open `pbt_simulator.ino` in Arduino IDE
2. Select Arduino Uno board
3. Select correct port (`/dev/ttyUSB0` or `/dev/ttyACM0`)
4. Click Upload

## Usage with PBT Tester

1. Upload this code to your Arduino
2. Connect hardware as shown above
3. Run `./start_tester.sh` on the Pi
4. Use the web interface at `http://localhost:5002`
5. Use "Generate Custom Peak" to create specific waveforms
6. Monitor waveforms on oscilloscope via Pin 3

## Integration
This Arduino code works with:
- `pbt_tester.py` - Main Pi application
- `start_tester.sh` - Pi startup script
- Web interface on `http://localhost:5002`

The Pi sends custom peak commands to this Arduino, which generates the corresponding waveforms and sends GPIO signals to the arcade machine.
