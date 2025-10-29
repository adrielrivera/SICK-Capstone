# Arduino Code Organization

This directory contains all Arduino sketches for the SICK Capstone project. Each sketch is in its own folder with exactly one `.ino` file.

## 📁 **Arduino Sketches**

### **1. PBT Simple** (`pbt_simple/`)
- **File**: `pbt_simple.ino`
- **Purpose**: Basic PBT sensor reading only (no GPIO control)
- **Use**: Testing sensor connection and data flow
- **Upload**: `./upload.sh`

### **2. PBT with GPIO** (`pbt_with_gpio/`)
- **File**: `pbt_with_gpio.ino`
- **Purpose**: PBT sensor reading + arcade GPIO control
- **Use**: Full PBT system with arcade integration
- **Upload**: `./upload.sh`

### **3. LiDAR Detection** (`lidar_detection/`)
- **File**: `lidar_detection.ino`
- **Purpose**: LiDAR monitoring and alarm system
- **Use**: Safety system with TiM100, TiM150, TiM240
- **Upload**: `./upload.sh`

## 🚀 **Quick Start**

### **Test PBT Sensor:**
```bash
cd pbt_simple
./upload.sh
```

### **Full PBT System:**
```bash
cd pbt_with_gpio
./upload.sh
```

### **LiDAR Safety System:**
```bash
cd lidar_detection
./upload.sh
```

## 🔧 **Hardware Connections**

### **PBT Sensor (All PBT sketches):**
- **Signal** → Arduino A0
- **Ground** → Arduino GND
- **Power** → 24V (external)

### **GPIO Control (pbt_with_gpio only):**
- **Arcade Pin 6** → Arduino Pin 6
- **Arcade Pin 5** → Arduino Pin 5

### **LiDAR Detection (lidar_detection only):**
- **TiM100** → Arduino Pin 8
- **TiM150** → Arduino Pin 9
- **TiM240** → Arduino Pin 10 (from Pi)
- **Buzzer** → Arduino Pin 11
- **Status LED** → Arduino Pin 13
- **Alarm LED** → Arduino Pin 12

## 📋 **Upload Requirements**

- **Arduino CLI** installed
- **Arduino Uno** connected via USB
- **Correct port** detected automatically
- **One .ino file per folder** (Arduino requirement)

## 🔍 **Troubleshooting**

1. **Compilation errors**: Check for multiple .ino files in same folder
2. **Upload errors**: Verify Arduino is connected and port is correct
3. **No data**: Check sensor connections and power supply
4. **GPIO not working**: Verify arcade connections and commands

Each sketch is self-contained and can be uploaded independently.
