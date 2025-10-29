#!/usr/bin/env python3
"""
LiDAR Detection Reader - Raspberry Pi
Reads TiM240 data and sends detection signals to LiDAR Arduino
Also reads status from LiDAR Arduino for webapp display
"""

import time
import serial
import socket
import threading
from collections import deque
import json

# TiM240 Configuration (from your existing code)
TIM240_IP = "192.168.0.20"
TIM240_PORT = 2111

# Arduino Configuration
ARDUINO_PORT = "/dev/ttyUSB1"  # Second Arduino (LiDAR detection)
ARDUINO_BAUD = 115200

# Detection parameters (from your existing TiM240 code)
ROI_MIN_DEG = 0
ROI_MAX_DEG = 180
D_CLOSE = 1000  # mm
D_PERSON_MIN = 200  # mm
D_PERSON_MAX = 800  # mm
DWELL_MS = 1000

class TiM240Reader:
    def __init__(self):
        self.sock = None
        self.connected = False
        self.detection_active = False
        self.last_detection_time = 0
        
    def connect(self):
        """Connect to TiM240 LiDAR."""
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((TIM240_IP, TIM240_PORT))
            self.connected = True
            print(f"✅ Connected to TiM240 at {TIM240_IP}:{TIM240_PORT}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to TiM240: {e}")
            self.connected = False
            return False
    
    def send_command(self, command):
        """Send command to TiM240."""
        if not self.connected:
            return False
        try:
            self.sock.send(command.encode())
            return True
        except Exception as e:
            print(f"Error sending command to TiM240: {e}")
            return False
    
    def read_data(self):
        """Read data from TiM240."""
        if not self.connected:
            return None
        try:
            data = self.sock.recv(1024)
            return data.decode('utf-8', errors='ignore')
        except Exception as e:
            print(f"Error reading from TiM240: {e}")
            return None
    
    def check_person_detection(self, data):
        """Check if person is detected in the data."""
        if not data or "DIST" not in data:
            return False
            
        try:
            # Parse distance data (simplified - you may need to adjust based on actual format)
            lines = data.split('\n')
            for line in lines:
                if line.startswith('DIST'):
                    # Extract distance values
                    parts = line.split()
                    if len(parts) > 1:
                        distances = [int(x) for x in parts[1:] if x.isdigit()]
                        
                        # Check for person in ROI
                        for i, dist in enumerate(distances):
                            angle = i * (180.0 / len(distances))  # Approximate angle
                            if ROI_MIN_DEG <= angle <= ROI_MAX_DEG:
                                if D_PERSON_MIN <= dist <= D_PERSON_MAX:
                                    return True
        except Exception as e:
            print(f"Error parsing TiM240 data: {e}")
        
        return False
    
    def close(self):
        """Close TiM240 connection."""
        if self.sock:
            self.sock.close()
            self.connected = False

class LiDARDetectionSystem:
    def __init__(self):
        self.tim240 = TiM240Reader()
        self.arduino = None
        self.running = False
        
        # Status tracking (simplified - only knows if ANY LiDAR is detecting)
        self.person_detected = False
        self.alarm_active = False
        
        # Status callbacks for webapp
        self.status_callbacks = []
    
    def add_status_callback(self, callback):
        """Add callback function for status updates."""
        self.status_callbacks.append(callback)
    
    def notify_status_change(self):
        """Notify all callbacks of status change."""
        status = {
            'person_detected': self.person_detected,
            'alarm_active': self.alarm_active
        }
        for callback in self.status_callbacks:
            try:
                callback(status)
            except Exception as e:
                print(f"Error in status callback: {e}")
    
    def connect_arduino(self):
        """Connect to LiDAR detection Arduino."""
        try:
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
            time.sleep(0.2)
            self.arduino.reset_input_buffer()
            print(f"✅ Connected to LiDAR Arduino at {ARDUINO_PORT}")
            return True
        except Exception as e:
            print(f"❌ Failed to connect to LiDAR Arduino: {e}")
            return False
    
    def read_arduino_status(self):
        """Read status from LiDAR Arduino."""
        if not self.arduino:
            return
        
        try:
            while self.arduino.in_waiting > 0:
                line = self.arduino.readline().decode(errors='ignore').strip()
                
                if line.startswith("LIDAR_STATUS:"):
                    # Parse status: person_detected,alarm_active
                    parts = line.split(":")[1].split(",")
                    if len(parts) >= 2:
                        self.person_detected = parts[0] == "1"
                        self.alarm_active = parts[1] == "1"
                        
                        self.notify_status_change()
                
                elif line.startswith("PERSON_DETECTED"):
                    print("🚨 Person detected by LiDAR system!")
                    self.notify_status_change()
                
                elif line.startswith("#"):
                    print(f"Arduino: {line}")
        
        except Exception as e:
            print(f"Error reading from Arduino: {e}")
    
    def send_tim240_detection(self, detected):
        """Send TiM240 detection signal to Arduino."""
        if not self.arduino:
            return
        
        try:
            if detected:
                self.arduino.write(b"TIM240_HIGH\n")
                print("📡 Sent TiM240 detection signal to Arduino")
            else:
                self.arduino.write(b"TIM240_LOW\n")
                print("📡 Sent TiM240 clear signal to Arduino")
        except Exception as e:
            print(f"Error sending TiM240 signal to Arduino: {e}")
    
    def run_tim240_reader(self):
        """Run TiM240 reading loop."""
        print("Starting TiM240 reader thread...")
        
        while self.running:
            if not self.tim240.connected:
                if not self.tim240.connect():
                    time.sleep(5)
                    continue
            
            # Read data from TiM240
            data = self.tim240.read_data()
            if data:
                # Check for person detection
                person_detected = self.tim240.check_person_detection(data)
                
                # Update detection state
                if person_detected != self.tim240_detected:
                    self.tim240_detected = person_detected
                    self.send_tim240_detection(person_detected)
                    self.notify_status_change()
            
            time.sleep(0.1)  # 10Hz update rate
    
    def run_arduino_reader(self):
        """Run Arduino reading loop."""
        print("Starting Arduino reader thread...")
        
        while self.running:
            self.read_arduino_status()
            time.sleep(0.1)  # 10Hz update rate
    
    def start(self):
        """Start the LiDAR detection system."""
        print("🚀 Starting LiDAR Detection System...")
        
        # Connect to Arduino
        if not self.connect_arduino():
            print("❌ Failed to start - Arduino not connected")
            return False
        
        # Connect to TiM240
        if not self.tim240.connect():
            print("⚠️  Warning - TiM240 not connected, continuing without it")
        
        self.running = True
        
        # Start reader threads
        tim240_thread = threading.Thread(target=self.run_tim240_reader, daemon=True)
        arduino_thread = threading.Thread(target=self.run_arduino_reader, daemon=True)
        
        tim240_thread.start()
        arduino_thread.start()
        
        print("✅ LiDAR Detection System started")
        return True
    
    def stop(self):
        """Stop the LiDAR detection system."""
        print("🛑 Stopping LiDAR Detection System...")
        self.running = False
        
        if self.arduino:
            self.arduino.close()
        
        self.tim240.close()
        print("✅ LiDAR Detection System stopped")

def main():
    """Main function for testing."""
    system = LiDARDetectionSystem()
    
    def status_callback(status):
        print(f"Status: {status}")
    
    system.add_status_callback(status_callback)
    
    try:
        if system.start():
            print("LiDAR Detection System running. Press Ctrl+C to stop.")
            while True:
                time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        system.stop()

if __name__ == "__main__":
    main()
