#!/bin/bash
# Start LiDAR Detection System (Arduino-only)

echo "=========================================="
echo "LiDAR Detection System Startup"
echo "=========================================="

# Check if required packages are installed
echo "Checking dependencies..."
python3 -c "import flask, flask_socketio, serial" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing required packages..."
    pip3 install flask flask-socketio pyserial
fi

# Check if Arduino is connected
echo "Checking Arduino connection..."
if [ ! -e "/dev/ttyUSB1" ] && [ ! -e "/dev/ttyACM1" ]; then
    echo "ERROR: No LiDAR detection Arduino found on /dev/ttyUSB1 or /dev/ttyACM1"
    echo "Please connect the LiDAR detection Arduino and try again"
    exit 1
fi

# Determine which port to use
if [ -e "/dev/ttyUSB1" ]; then
    ARDUINO_PORT="/dev/ttyUSB1"
elif [ -e "/dev/ttyACM1" ]; then
    ARDUINO_PORT="/dev/ttyACM1"
fi

echo "✅ Arduino found on $ARDUINO_PORT"

# Start the LiDAR webapp
echo "Starting LiDAR Detection System..."
echo "Arduino port: $ARDUINO_PORT"
echo "Web interface: http://localhost:5001"
echo "Press Ctrl+C to stop"
echo "=========================================="

python3 lidar_webapp.py
