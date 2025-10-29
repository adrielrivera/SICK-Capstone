#!/bin/bash
# Upload script for SICK7 PBT-Only Arduino code

echo "=========================================="
echo "SICK7 PBT-Only Arduino Upload"
echo "=========================================="

# Change to the directory containing the .ino file
cd "$(dirname "$0")"

# Check if Arduino CLI is installed
if ! command -v arduino-cli &> /dev/null; then
    echo "ERROR: Arduino CLI not found!"
    echo "Please install Arduino CLI first:"
    echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
    echo "  arduino-cli core install arduino:avr"
    exit 1
fi

# Detect Arduino port
ARDUINO_PORT=""
if [ -e "/dev/ttyUSB0" ]; then
    ARDUINO_PORT="/dev/ttyUSB0"
elif [ -e "/dev/ttyACM0" ]; then
    ARDUINO_PORT="/dev/ttyACM0"
else
    echo "ERROR: No Arduino detected!"
    echo "Please connect Arduino and try again"
    exit 1
fi

echo "Arduino detected on: $ARDUINO_PORT"

# Compile the sketch
echo "Compiling PBT-Only Arduino code..."
arduino-cli compile --fqbn arduino:avr:uno .

if [ $? -ne 0 ]; then
    echo "ERROR: Compilation failed!"
    exit 1
fi

# Upload the sketch
echo "Uploading to Arduino..."
arduino-cli upload -p $ARDUINO_PORT --fqbn arduino:avr:uno .

if [ $? -eq 0 ]; then
    echo "=========================================="
    echo "SUCCESS: PBT-Only Arduino code uploaded!"
    echo "Arduino is now ready for PBT sensor reading"
    echo "=========================================="
else
    echo "ERROR: Upload failed!"
    exit 1
fi
