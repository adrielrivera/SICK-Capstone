# Unified PBT + LiDAR Arduino Code

## Overview
This Arduino sketch combines PBT sensor processing and LiDAR detection in a single system. It's designed to work with the SICK7 Pi application for real-time PBT scoring with safety monitoring.

## Hardware Connections
```
PBT Sensor    → A0 (Analog input)
TiM100 (Left) → Pin 8 (Digital input with pull-down resistor)
TiM150 (Right)→ Pin 9 (Digital input with pull-down resistor)
LED Status    → Pin 13 (Built-in LED)
Arcade Pin 5  → Pin 5 (ACTIVE signal)
Arcade Pin 6  → Pin 6 (START signal)
```

## Features
- **PBT Sensor**: Real-time analog reading with auto-calibration
- **LiDAR Detection**: TiM100 (Left) and TiM150 (Right) monitoring
- **GPIO Control**: Arcade motherboard control via serial commands
- **Status Reporting**: Combined status updates every 500ms
- **Safety Integration**: LiDAR status sent to Pi for safety system

## Upload Instructions

### Using Arduino CLI (Recommended)
```bash
cd ~/SICK-Capstone/SICK-Capstone/arduino_stuff/unified_pbt_lidar
./upload.sh
```

### Using Arduino IDE
1. Open `unified_pbt_lidar.ino` in Arduino IDE
2. Select Arduino Uno board
3. Select correct port (`/dev/ttyUSB0` or `/dev/ttyACM0`)
4. Click Upload

## Output Format
- **PBT Data**: Raw ADC values (512, 513, 514...)
- **LiDAR Status**: `# LIDAR_STATUS: TIM100=CLEAR TIM150=DETECTED`
- **Detection Events**: `# TiM100 DETECTED - Person on LEFT side`
- **GPIO Commands**: Responds to `PIN5_HIGH`, `PIN6_LOW`, etc.

## Integration with Pi
This Arduino code is designed to work with:
- `app_combined.py` - Main Pi application
- `start_combined.sh` - Pi startup script
- Web interface on `http://localhost:5000`

## Troubleshooting
- **Compilation errors**: Make sure only one `.ino` file in directory
- **Upload fails**: Check Arduino connection and port
- **No data**: Verify serial connection and baud rate (115200)
- **LiDAR not working**: Check pin connections and pull-down resistors
