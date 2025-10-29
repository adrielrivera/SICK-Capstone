# GPIO Output and LiDAR System Updates

## Changes Made

### 1. TiM240 GPIO Output (`tim240_run.py`)
- **Added GPIO support**: Import `RPi.GPIO` library
- **GPIO Pin**: Pin 18 (GPIO18) outputs 3.3V when person detected
- **Logic**: `GPIO.HIGH` when `state == "ALERT_REAR"`, `GPIO.LOW` otherwise
- **Cleanup**: Proper GPIO cleanup on exit

### 2. Simplified LiDAR System (`start_lidar_system.sh`)
- **Removed TiM240 logic**: No longer checks TiM240 connectivity
- **Arduino-only**: Only reads from Arduino at `/dev/ttyUSB1` or `/dev/ttyACM1`
- **Error handling**: Exits if no Arduino found

### 3. Arduino-Only Reader (`lidar_detection_reader.py`)
- **Removed TiM240 code**: No more TiM240 connection or processing
- **Simplified**: Only reads from Arduino at 9600 baud
- **Auto-detection**: Tries multiple ports (`/dev/ttyUSB1`, `/dev/ttyACM1`, etc.)

## Hardware Connections

### TiM240 System (Pi GPIO Output)
- **TiM240 Ethernet** → Pi Ethernet (192.168.0.20:2111)
- **Pi GPIO18** → OR Gate Input 3 (3.3V when person detected)

### LiDAR Detection Arduino
- **OR Gate Output** → Arduino Pin 8 (TiM100 OR TiM150 OR TiM240)
- **Arduino USB** → Pi `/dev/ttyUSB1` (9600 baud)
- **Buzzer** → Arduino Pin 9
- **LED** → Arduino Pin 13

### PBT System (Separate Arduino)
- **PBT Sensor** → Arduino A0
- **Arduino USB** → Pi `/dev/ttyUSB0` (115200 baud)
- **Arcade Pins** → Arduino D5, D6

## Usage

### Start TiM240 with GPIO Output
```bash
cd /path/to/pi_stuff
python3 tim240_run.py
```
- Outputs 3.3V on GPIO18 when person detected
- Sends signal to OR gate for LiDAR Arduino

### Start LiDAR System (Arduino-only)
```bash
cd /path/to/pi_stuff
./start_lidar_system.sh
```
- Reads from Arduino at `/dev/ttyUSB1`
- Web interface: http://localhost:5001

### Test LiDAR System
```bash
cd /path/to/pi_stuff
python3 test_lidar_system.py
```

## Status Messages

Arduino sends to Pi:
- `LIDAR_STATUS:1,0` - Person detected, alarm not active
- `LIDAR_STATUS:0,1` - No person, alarm active
- `PERSON_DETECTED` - Person detection event
- `# Ready` - System ready messages

## Notes

- TiM240 handles rear detection and outputs GPIO signal
- LiDAR Arduino handles all detection inputs via OR gate
- Pi reads Arduino status for webapp display
- No TiM240 logic in LiDAR webapp system
