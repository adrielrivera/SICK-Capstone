#!/bin/bash
# Upload script for SICK7 PBT Simulator Arduino Code
# Uploads pbt_simulator.ino to Arduino for PBT testing with waveform generation

echo "=========================================="
echo "Uploading SICK7 PBT Simulator Arduino Code"
echo "=========================================="

# Check for arduino-cli
if ! command -v arduino-cli &> /dev/null
then
    echo "ERROR: arduino-cli not found!"
    echo "Please install Arduino CLI first:"
    echo "  curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh"
    echo "  export PATH=\"\$PATH:~/bin\" # Or wherever arduino-cli was installed"
    exit 1
fi

# Detect Arduino port
ARDUINO_PORT=""
if ls /dev/ttyUSB* 1> /dev/null 2>&1; then
    ARDUINO_PORT=$(ls /dev/ttyUSB* | head -n 1)
elif ls /dev/ttyACM* 1> /dev/null 2>&1; then
    ARDUINO_PORT=$(ls /dev/ttyACM* | head -n 1)
fi

if [ -z "$ARDUINO_PORT" ]; then
    echo "ERROR: No Arduino detected on /dev/ttyUSB0 or /dev/ttyACM0"
    echo "Please ensure your Arduino is connected."
    exit 1
fi

echo "Found Arduino on: $ARDUINO_PORT"

# Compile the code
echo "Compiling PBT simulator Arduino code..."
# Compile the sketch in the current directory
arduino-cli compile --fqbn arduino:avr:uno .

if [ $? -ne 0 ]; then
    echo "ERROR: Compilation failed!"
    echo "Make sure there's only one .ino file in the directory"
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
echo "Upload complete! Arduino is running PBT simulator."
echo "=========================================="
echo ""
echo "Hardware connections:"
echo "  Oscilloscope Channel 1 → Arduino Pin 3 (PBT Waveform)"
echo "  Oscilloscope Channel 2 → Arduino Pin 6 (Arcade Start Signal)"
echo "  Oscilloscope Channel 3 → Arduino Pin 5 (Arcade Active Signal)"
echo "  Oscilloscope Ground → Arduino GND"
echo ""
echo "Arcade connections:"
echo "  Arcade Pin 5 ──── Arduino Pin 5 (ACTIVE signal)"
echo "  Arcade Pin 6 ──── Arduino Pin 6 (START signal)"
echo "  Arcade GND  ──── Arduino GND"
echo ""
echo "Features:"
echo "  • Generates realistic PBT waveforms"
echo "  • Custom peak generation via serial commands"
echo "  • GPIO control for arcade machine"
echo "  • LiDAR detection (TiM100/TiM150)"
echo "  • Oscilloscope monitoring on Pin 3"
echo ""
echo "Now you can run: ./start_tester.sh"
echo "=========================================="
