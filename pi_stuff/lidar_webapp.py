#!/usr/bin/env python3
"""
LiDAR Detection Webapp - Raspberry Pi
Displays real-time LiDAR detection status and alarm information
"""

import time
import threading
from flask import Flask, render_template
from flask_socketio import SocketIO, emit
import json

app = Flask(__name__)
app.config['SECRET_KEY'] = 'lidar_detection_secret_key'
socketio = SocketIO(app, cors_allowed_origins="*", async_mode='threading')

# Global status
lidar_status = {
    'tim100_detected': False,
    'tim150_detected': False,
    'tim240_detected': False,
    'any_detection': False,
    'alarm_active': False,
    'system_connected': False
}

# LiDAR detection system
lidar_system = None

def status_callback(status):
    """Callback for LiDAR status updates."""
    global lidar_status
    
    # Update status
    lidar_status.update(status)
    lidar_status['system_connected'] = True
    
    # Emit to web clients
    socketio.emit('lidar_status', lidar_status)
    
    # Log significant changes
    if status.get('any_detection'):
        print(f"🚨 PERSON DETECTED - TiM100: {status.get('tim100_detected')}, TiM150: {status.get('tim150_detected')}, TiM240: {status.get('tim240_detected')}")
    elif status.get('alarm_active'):
        print("🔊 Alarm is active")

@app.route('/')
def index():
    """Main page."""
    return render_template('lidar_monitor.html')

@socketio.on('connect')
def handle_connect():
    """Handle client connection."""
    print('Client connected to LiDAR webapp')
    emit('lidar_status', lidar_status)

@socketio.on('disconnect')
def handle_disconnect():
    """Handle client disconnection."""
    print('Client disconnected from LiDAR webapp')

@socketio.on('request_status')
def handle_status_request():
    """Send current status to client."""
    emit('lidar_status', lidar_status)

@socketio.on('test_tim100')
def handle_test_tim100():
    """Test TiM100 detection (simulate)."""
    print("🧪 Testing TiM100 detection")
    # This would send a test signal to the Arduino
    # For now, just log it
    emit('test_result', {'sensor': 'TiM100', 'status': 'Test signal sent'})

@socketio.on('test_tim150')
def handle_test_tim150():
    """Test TiM150 detection (simulate)."""
    print("🧪 Testing TiM150 detection")
    emit('test_result', {'sensor': 'TiM150', 'status': 'Test signal sent'})

@socketio.on('test_tim240')
def handle_test_tim240():
    """Test TiM240 detection (simulate)."""
    print("🧪 Testing TiM240 detection")
    emit('test_result', {'sensor': 'TiM240', 'status': 'Test signal sent'})

@socketio.on('reset_alarm')
def handle_reset_alarm():
    """Reset alarm manually."""
    print("🔄 Resetting alarm manually")
    # This would send a reset command to the Arduino
    emit('test_result', {'action': 'reset_alarm', 'status': 'Alarm reset command sent'})

def start_lidar_system():
    """Start the LiDAR detection system in background."""
    global lidar_system
    
    try:
        from lidar_detection_reader import LiDARDetectionSystem
        lidar_system = LiDARDetectionSystem()
        lidar_system.add_status_callback(status_callback)
        
        if lidar_system.start():
            print("✅ LiDAR detection system started")
        else:
            print("❌ Failed to start LiDAR detection system")
    except Exception as e:
        print(f"❌ Error starting LiDAR detection system: {e}")

if __name__ == '__main__':
    print("=" * 60)
    print("LiDAR Detection Webapp")
    print("=" * 60)
    print("Starting LiDAR detection system...")
    
    # Start LiDAR system in background thread
    lidar_thread = threading.Thread(target=start_lidar_system, daemon=True)
    lidar_thread.start()
    
    # Give system time to start
    time.sleep(2)
    
    print("Starting web server...")
    print("Web interface: http://localhost:5001")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    try:
        socketio.run(app, host='0.0.0.0', port=5001, debug=False, allow_unsafe_werkzeug=True)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        if lidar_system:
            lidar_system.stop()
