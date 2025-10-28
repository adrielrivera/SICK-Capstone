#!/bin/bash
# Upload unified Arduino code for PBT + LiDAR system

echo "=========================================="
echo "Uploading SICK7 Unified Arduino Code"
echo "=========================================="

# Check if Arduino CLI is installed
if ! command -v arduino-cli &> /dev/null; then
    echo "ERROR: arduino-cli not found!"
    echo "Please install Arduino CLI first:"
    echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
    echo "  export PATH=$PATH:~/bin"
    exit 1
fi

# Check if Arduino is connected
echo "Checking for Arduino..."
if [ ! -e "/dev/ttyUSB0" ] && [ ! -e "/dev/ttyACM0" ]; then
    echo "ERROR: No Arduino detected!"
    echo "Please connect Arduino and try again"
    echo "Expected ports: /dev/ttyUSB0 or /dev/ttyACM0"
    exit 1
fi

# Determine Arduino port
if [ -e "/dev/ttyUSB0" ]; then
    ARDUINO_PORT="/dev/ttyUSB0"
else
    ARDUINO_PORT="/dev/ttyACM0"
fi

echo "Found Arduino on: $ARDUINO_PORT"

# Compile the code
echo "Compiling unified Arduino code..."
arduino-cli compile --fqbn arduino:avr:uno .

if [ $? -ne 0 ]; then
    echo "ERROR: Compilation failed!"
    exit 1
fi

echo "Compilation successful!"

# Upload to Arduino
echo "Uploading to Arduino..."
arduino-cli upload -p $ARDUINO_PORT --fqbn arduino:avr:uno .

if [ $? -ne 0 ]; then
    echo "ERROR: Upload failed!"
    exit 1
fi

echo "=========================================="
echo "Upload successful!"
echo "Arduino is now running unified PBT + LiDAR code"
echo "You can now run the Pi system:"
echo "  cd ~/SICK/SICK-App"
echo "  ./start_combined.sh"
echo "=========================================="
