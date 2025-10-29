#!/bin/bash
# Arduino Upload Script for LiDAR Detection System

ARDUINO_DIR="$(dirname "$0")"
SKETCH_NAME="lidar_detection.ino"
FQBN="arduino:avr:uno" # Fully Qualified Board Name for Arduino Uno

echo "=========================================="
echo "LiDAR Detection Arduino Upload Script"
echo "=========================================="

# Find Arduino port
ARDUINO_PORT=$(ls /dev/ttyACM* /dev/ttyUSB* 2>/dev/null | head -n 1)

if [ -z "$ARDUINO_PORT" ]; then
    echo "ERROR: No Arduino device found. Please ensure it's connected."
    exit 1
fi

echo "Found Arduino at: $ARDUINO_PORT"
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
