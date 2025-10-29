#!/bin/bash
# Start LiDAR Detection System

echo "=========================================="
echo "LiDAR Detection System Startup"
echo "=========================================="

# Check if required packages are installed
echo "Checking dependencies..."
python3 -c "import flask, flask_socketio, serial, socket" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "Installing required packages..."
    pip3 install flask flask-socketio pyserial
fi

# Check if Arduino is connected
echo "Checking Arduino connection..."
if [ ! -e "/dev/ttyUSB1" ] && [ ! -e "/dev/ttyACM1" ]; then
    echo "WARNING: No second Arduino detected on /dev/ttyUSB1 or /dev/ttyACM1"
    echo "Make sure LiDAR detection Arduino is connected"
fi

# Check if TiM240 is reachable
echo "Checking TiM240 connection..."
ping -c 1 192.168.0.20 > /dev/null 2>&1
if [ $? -eq 0 ]; then
    echo "✅ TiM240 is reachable"
else
    echo "⚠️  TiM240 not reachable - continuing without it"
fi

# Start the LiDAR webapp
echo "Starting LiDAR Detection System..."
echo "Web interface: http://localhost:5001"
echo "Press Ctrl+C to stop"
echo "=========================================="

python3 lidar_webapp.py
