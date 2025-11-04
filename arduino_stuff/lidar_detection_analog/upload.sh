#!/bin/bash
# Arduino Upload Script for LiDAR Detection System (Analog Input Version)

ARDUINO_DIR="$(dirname "$0")"
SKETCH_NAME="lidar_detection_analog.ino"
FQBN="arduino:avr:uno" # Fully Qualified Board Name for Arduino Uno

echo "=========================================="
echo "LiDAR Detection Arduino Upload Script (Analog)"
echo "=========================================="

# Find Arduino port (prioritize ttyUSB1 for LiDAR Arduino)
if [ -e "/dev/ttyUSB1" ]; then
    ARDUINO_PORT="/dev/ttyUSB1"
    echo "Using LiDAR Arduino port: $ARDUINO_PORT"
else
    ARDUINO_PORT=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n 1)
    if [ -z "$ARDUINO_PORT" ]; then
        echo "ERROR: No Arduino device found. Please ensure it's connected."
        exit 1
    fi
    echo "Found Arduino at: $ARDUINO_PORT"
fi

echo "Compiling and uploading $SKETCH_NAME..."

# Navigate to the sketch directory
cd "$ARDUINO_DIR" || { echo "ERROR: Could not change to Arduino directory."; exit 1; }

# Compile and upload
arduino-cli compile --fqbn "$FQBN" .
if [ $? -ne 0 ]; then
    echo "ERROR: Arduino compilation failed."
    exit 1
fi

arduino-cli upload -p "$ARDUINO_PORT" --fqbn "$FQBN" .
if [ $? -ne 0 ]; then
    echo "ERROR: Arduino upload failed."
    exit 1
fi

echo "SUCCESS: $SKETCH_NAME uploaded to Arduino."
echo "=========================================="
echo "NOTE: Uses analog pin A0 with 0.8V threshold (~165 ADC counts)"
