# SICK7 - PBT Sensor Safety System

A complete safety monitoring system for a PBT (Photo Beam Transmitter) sensor-based arcade game with LiDAR-based person detection and credit tracking.

## System Overview

The system consists of:
- **PBT Arduino** (`/dev/ttyUSB0`): Reads PBT sensor, controls arcade GPIO, tracks PBT hits
- **LiDAR Arduino** (`/dev/ttyUSB1`): Monitors side LiDARs (TiM100/TiM150) and TiM240, controls alarm
- **Raspberry Pi**: Runs webapp, processes TiM240 LiDAR data, coordinates system
- **Web Dashboard**: Real-time visualization of sensor data, safety status, and credit tracking

## Hardware Setup

### Arduino Connections

#### PBT Arduino (Port: `/dev/ttyUSB0`)
- **PBT Sensor**: Connected to Analog Pin A0
- **Arcade GPIO Pin 6**: Arduino Digital Pin 6 (Press START)
- **Arcade GPIO Pin 5**: Arduino Digital Pin 5 (Press ACTIVE)
- **Credit Add Signal**: Pi GPIO → Arduino Digital Pin 2 (falling edge interrupt)
- **Status LED**: Pin 13

#### LiDAR Arduino (Port: `/dev/ttyUSB1`)
- **OR Gate Input**: Analog Pin A0 (5V signal from TiM100 OR TiM150)
- **TiM240 Input**: Digital Pin 3 (3.3V signal from Pi GPIO 17)
- **Credit Add Signal**: Pi GPIO 18 → Digital Pin 2 (falling edge interrupt)
- **Buzzer**: Pin 9
- **Status LED**: Pin 13

### Raspberry Pi GPIO Connections
- **GPIO 17**: TiM240 detection output (HIGH when person detected) → LiDAR Arduino Pin 3
- **GPIO 18**: Credit add signal (falling edge) → Both Arduinos Pin 2

### LiDAR Setup
- **TiM100/TiM150**: Side LiDARs → OR gate circuit → LiDAR Arduino A0
- **TiM240**: Ethernet-connected rear LiDAR → Pi processes data → GPIO 17 output

## Software Installation

### 1. Install Python Dependencies

```bash
cd SICK-App
pip3 install -r requirements.txt
```

Or install manually:
```bash
pip3 install Flask==3.0.0 Flask-SocketIO==5.3.5 pyserial==3.5
```

### 2. Upload Arduino Sketches

#### PBT Arduino (`/dev/ttyUSB0`)
```bash
cd ../SICK-Capstone/SICK-Capstone/arduino_stuff/pbt_with_credits
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno pbt_with_credits.ino
```

#### LiDAR Arduino (`/dev/ttyUSB1`)
```bash
cd ../SICK-Capstone/SICK-Capstone/arduino_stuff/lidar_detection_dual
arduino-cli upload -p /dev/ttyUSB1 --fqbn arduino:avr:uno lidar_detection_dual.ino
```

### 3. Configure Serial Ports

Edit `SICK-App/config.py` if your Arduino ports differ:
```python
SERIAL_PORT = "/dev/ttyUSB0"  # PBT Arduino
LIDAR_SERIAL_PORT = "/dev/ttyUSB1"  # LiDAR Arduino
```

## Running the System

### 1. Start TiM240 LiDAR Processing (Terminal 1)

```bash
cd SICK-Capstone/SICK-Capstone/pi_stuff
python3 tim240_gpio_test.py
```

This script:
- Connects to TiM240 LiDAR via Ethernet
- Analyzes scan data for person detection
- Outputs GPIO 17 HIGH when person detected
- Suppresses false alarms from hammer (objects < 200cm)

### 2. Start Web Application (Terminal 2)

**With Credit Tracking:**
```bash
cd SICK7/SICK-App
./start_with_credits.sh
```

**Without Credit Tracking (simpler):**
```bash
./start_combined_dual.sh
```

The webapp will be available at: `http://localhost:5000` (or your Pi's IP address)

### 3. Access Web Dashboard

Open a web browser and navigate to:
- Local: `http://localhost:5000`
- Remote: `http://<pi-ip-address>:5000`

## System Features

### Credit Tracking System
- **2 PBT hits = 1 credit deducted**
- Credits start at 0 (add credits via GPIO signal or serial command)
- LiDAR safety is **disabled when credits == 0** (no alarm even if person detected)
- Credit balance displayed in web dashboard

### Safety System
- **Side LiDARs** (TiM100/TiM150): OR gate detects person on left or right
- **Rear LiDAR** (TiM240): Kite-shaped detection field, hammer suppression
- **Combined Detection**: Any LiDAR detecting = alarm triggered (if credits > 0)
- **Alarm Duration**: 5 seconds buzzer + LED

### PBT Sensor Processing
- **Sample Rate**: 800 Hz
- **Peak Detection**: Envelope-based with configurable threshold
- **Pulse Generation**: Proportional to hit strength (30-100 ADC → 10-100ms pulse)
- **GPIO Control**: Automatically presses arcade buttons based on hit strength

### Web Dashboard
- Real-time waveform visualization
- Safety status banner (AREA CLEAR / PERSON DETECTED)
- Credit balance display
- PBT hit history (last 10 hits with ADC values)
- Connection status indicators

## Configuration

### PBT Sensor Parameters (`config.py`)
```python
TRIGGER_THRESHOLD = 30        # Peak detection threshold (ADC counts)
ENVELOPE_ALPHA = 0.12         # Envelope smoothing (0-1)
BASELINE_ALPHA = 0.001        # Baseline tracking (0-1)
```

### TiM240 Detection Parameters (`tim240_gpio_test.py`)
```python
KITE_MAX_ANGLE = 20.0715      # degrees
KITE_MAX_DISTANCE = 209.932   # cm at max angle
KITE_CENTER_DISTANCE = 247.0  # cm at 0 degrees
HAMMER_MAX_DISTANCE = 200.0   # cm (objects closer = hammer, suppressed)
```

### Credit System
- **Hits per Credit**: 2 (hardcoded in Arduino and Pi)
- **Debounce Time**: 1000ms (max 1 credit add per second)

## Troubleshooting

### Arduino Not Detected
```bash
# Check if devices are connected
ls /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0
sudo chmod 666 /dev/ttyUSB1
```

### Serial Communication Errors
- Verify baud rates match:
  - PBT Arduino: 115200
  - LiDAR Arduino: 9600
- Check serial port permissions
- Ensure only one process is accessing each port

### TiM240 Not Detecting
- Verify Ethernet connection to TiM240
- Check IP address in `tim240_gpio_test.py`
- Monitor GPIO 17 output: `gpio readall` or oscilloscope
- Check for "HAMMER_SUPPRESS" messages (objects too close)

### Web Dashboard Not Updating
- Check browser console (F12) for JavaScript errors
- Verify SocketIO connection in browser console
- Check Python console for serial communication errors
- Ensure both Arduinos are connected and sending data

### Alarm Not Triggering
- Verify credits > 0 (alarm disabled when credits == 0)
- Check LiDAR Arduino serial output for detection messages
- Verify GPIO connections (OR gate, TiM240 signal)
- Check buzzer/LED connections on LiDAR Arduino

## Serial Commands

### LiDAR Arduino
- `STATUS`: Get current status and credit balance
- `ADD_CREDIT`: Add 1 credit (for testing)
- `RESET_CREDITS`: Reset credits to 0

### PBT Arduino
- Commands sent automatically by Pi (GPIO control)

## File Structure

```
SICK7/
├── README.md                    # This file
├── SICK-App/
│   ├── app_with_credits.py      # Main webapp (with credit tracking)
│   ├── app_combined.py          # Main webapp (without credits)
│   ├── config.py                # Configuration file
│   ├── requirements.txt         # Python dependencies
│   ├── start_with_credits.sh    # Start script (with credits)
│   ├── start_combined_dual.sh   # Start script (without credits)
│   ├── static/                  # Web assets (CSS, JS)
│   └── templates/              # HTML templates
│
SICK-Capstone/
└── SICK-Capstone/
    ├── arduino_stuff/
    │   ├── pbt_with_credits/    # PBT Arduino sketch
    │   └── lidar_detection_dual/ # LiDAR Arduino sketch
    └── pi_stuff/
        ├── tim240_gpio_test.py  # TiM240 LiDAR processing
        └── fix_gpio_permissions.sh
```

## System Architecture

```
┌─────────────┐
│  TiM240     │──Ethernet──► Pi (tim240_gpio_test.py) ──GPIO 17──►
│  (Rear)     │                                            │
└─────────────┘                                            │
                                                           ▼
┌─────────────┐         ┌──────────────┐         ┌─────────────────┐
│ TiM100/150  │──OR──► │ LiDAR Arduino│◄──GPIO──┤  Pi Webapp      │
│  (Sides)    │        │  (ttyUSB1)   │         │ (app_with_      │
└─────────────┘        └──────────────┘         │  credits.py)     │
                              │                  └─────────────────┘
                              │                           │
                              ▼                           │
                        ┌──────────┐                     │
                        │  Alarm   │                     │
                        │ (Buzzer) │                     │
                        └──────────┘                     │
                                                          │
┌─────────────┐         ┌──────────────┐                 │
│  PBT Sensor │──────► │  PBT Arduino │◄──Serial────────┘
└─────────────┘        │  (ttyUSB0)   │
                       └──────────────┘
                              │
                              ▼
                        ┌──────────┐
                        │ Arcade   │
                        │  GPIO    │
                        └──────────┘
```

## License

This project is part of a capstone project.

## Support

For issues or questions, check:
1. Serial console output from both Arduinos
2. Python console output from webapp
3. Browser console (F12) for webapp errors
4. GPIO status: `gpio readall`

