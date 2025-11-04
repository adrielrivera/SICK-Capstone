#!/bin/bash
# Arduino Upload Script for SEC110 Trigger Test (Continuous)

ARDUINO_DIR="$(dirname "$0")"
SKETCH_NAME="sec110_test.ino"
FQBN="arduino:avr:uno"

echo "=========================================="
echo "SEC110 Trigger Test (Continuous) - Upload"
echo "=========================================="

# Find Arduino port
ARDUINO_PORT=$(ls /dev/ttyUSB0 /dev/ttyUSB1 /dev/ttyACM* 2>/dev/null | head -n 1)

if [ -z "$ARDUINO_PORT" ]; then
    echo "ERROR: No Arduino device found."
    exit 1
fi

echo "Found Arduino at: $ARDUINO_PORT"
cd "$ARDUINO_DIR" || { echo "ERROR: Could not change directory."; exit 1; }

arduino-cli compile --fqbn "$FQBN" .
if [ $? -ne 0 ]; then
    echo "ERROR: Compilation failed."
    exit 1
fi

arduino-cli upload -p "$ARDUINO_PORT" --fqbn "$FQBN" .
if [ $? -ne 0 ]; then
    echo "ERROR: Upload failed."
    exit 1
fi

echo "SUCCESS: Uploaded. Pin 7 toggles LOW→HIGH every 2 seconds"
echo "=========================================="

