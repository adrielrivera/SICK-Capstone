# LiDAR Detection System

A comprehensive LiDAR monitoring system that detects people using TiM100, TiM150, and TiM240 sensors, with real-time alarm activation and web monitoring.

## 🏗️ **System Architecture**

```
TiM240 (Rear) → Raspberry Pi → GPIO Pin → Arduino → Alarm Circuit
TiM100 (Left) ──────────────────────────→ Arduino → Alarm Circuit  
TiM150 (Right) ─────────────────────────→ Arduino → Alarm Circuit
```

### **Components:**
1. **TiM240 LiDAR** - Connected to Pi via Ethernet (192.168.0.20:2111)
2. **TiM100/TiM150 LiDARs** - Connected directly to Arduino digital inputs
3. **Second Arduino** - LiDAR detection and alarm control
4. **Raspberry Pi** - TiM240 processing and web interface
5. **Alarm Circuit** - Piezo buzzer and LED indicators

## 🔧 **Hardware Connections**

### **Arduino (LiDAR Detection):**
- **TiM100 Signal** → Pin 8 (with pull-down resistor)
- **TiM150 Signal** → Pin 9 (with pull-down resistor)
- **TiM240 Signal** → Pin 10 (from Pi GPIO)
- **Piezo Buzzer** → Pin 11
- **Status LED** → Pin 13 (blinks when running)
- **Alarm LED** → Pin 12 (blinks during alarm)
- **GND** → Common ground

### **Raspberry Pi:**
- **TiM240** → Ethernet (192.168.0.20:2111)
- **Arduino** → USB (/dev/ttyUSB1 or /dev/ttyACM1)

## 🚀 **Setup Instructions**

### **1. Upload Arduino Code:**
```bash
cd ~/SICK-Capstone/SICK-Capstone/arduino_stuff/lidar_detection
./upload.sh
```

### **2. Start LiDAR System:**
```bash
cd ~/SICK-Capstone/SICK-Capstone/pi_stuff
./start_lidar_system.sh
```

### **3. Access Web Interface:**
Open browser to: `http://localhost:5001`

## 📊 **Features**

### **Real-time Monitoring:**
- **TiM100 Detection** - Left side monitoring
- **TiM150 Detection** - Right side monitoring  
- **TiM240 Detection** - Rear monitoring via Pi
- **Combined Status** - Any detection triggers alarm

### **Alarm System:**
- **5-second alarm** when person detected
- **Sweeping tone** (500-1000Hz) on piezo buzzer
- **Blinking LED** during alarm
- **Web notifications** and status display

### **Web Interface:**
- **Live sensor status** with visual indicators
- **Alarm status** with clear notifications
- **Test buttons** for each sensor
- **System log** with timestamped events
- **Manual alarm reset** capability

## 🔍 **Detection Logic**

### **TiM100/TiM150:**
- Direct digital input monitoring
- HIGH = Person detected
- LOW = Area clear
- Immediate alarm trigger

### **TiM240:**
- Ethernet communication with Pi
- Distance-based detection (200-800mm)
- ROI filtering (0-180 degrees)
- Pi sends detection signal to Arduino

## 🛠️ **Troubleshooting**

### **Arduino Not Detected:**
```bash
ls /dev/ttyUSB* /dev/ttyACM*
# Should show /dev/ttyUSB1 or /dev/ttyACM1
```

### **TiM240 Connection Issues:**
```bash
ping 192.168.0.20
# Should respond if TiM240 is reachable
```

### **Web Interface Not Loading:**
- Check if port 5001 is available
- Verify Flask dependencies are installed
- Check system logs for errors

### **No Alarm Sound:**
- Verify buzzer is connected to Pin 11
- Check Arduino serial output for detection signals
- Test with manual sensor simulation

## 📝 **Serial Commands**

### **Arduino Commands:**
- `STATUS` - Get current detection status
- `RESET_ALARM` - Manually reset alarm
- `TIM240_HIGH` - Simulate TiM240 detection
- `TIM240_LOW` - Simulate TiM240 clear

### **Status Format:**
```
LIDAR_STATUS:tim100,tim150,tim240,any_detection,alarm_active
Example: LIDAR_STATUS:1,0,1,1,1
```

## 🔄 **Integration with PBT System**

This LiDAR system is designed to work alongside the PBT scoring system:

1. **LiDAR detects person** → Alarm activates
2. **Arduino sends signal to Pi** → PBT system disabled
3. **Web interface shows warning** → User notified
4. **Person leaves** → Alarm stops, PBT system re-enabled

## 📋 **File Structure**

```
pi_stuff/
├── lidar_detection_reader.py    # TiM240 reader and Arduino communication
├── lidar_webapp.py              # Flask web application
├── start_lidar_system.sh        # Startup script
├── templates/
│   └── lidar_monitor.html       # Web interface
└── LIDAR_SYSTEM_README.md       # This file

arduino_stuff/lidar_detection/
├── lidar_detection.ino          # Arduino code
├── upload.sh                    # Upload script
└── README.md                    # Arduino documentation
```

## 🎯 **Expected Behavior**

1. **System starts** → All sensors show "CLEAR"
2. **Person detected** → Sensor shows "DETECTED", alarm activates
3. **5-second alarm** → Buzzer sounds, LED blinks
4. **Person leaves** → Sensor shows "CLEAR", alarm stops
5. **Web interface** → Real-time updates and logging

This system provides comprehensive LiDAR monitoring with immediate alarm response and detailed web-based status tracking.
