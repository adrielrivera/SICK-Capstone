#!/usr/bin/env python3
"""
LiDAR Detection Reader - Raspberry Pi
Reads status from LiDAR Arduino for webapp display
"""

import time
import serial
import threading
import os

# Arduino Configuration
ARDUINO_PORT = "/dev/ttyUSB1"  # Second Arduino (LiDAR detection)
ARDUINO_BAUD = 9600  # Match the lidar_detection.ino baud rate

class LiDARDetectionSystem:
    def __init__(self):
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
        # Try different possible ports
        possible_ports = ["/dev/ttyUSB1", "/dev/ttyACM1", "/dev/ttyUSB0", "/dev/ttyACM0"]
        
        for port in possible_ports:
            if os.path.exists(port):
                try:
                    self.arduino = serial.Serial(port, ARDUINO_BAUD, timeout=1)
                    time.sleep(0.2)
                    self.arduino.reset_input_buffer()
                    print(f"✅ Connected to LiDAR Arduino at {port}")
                    return True
                except Exception as e:
                    print(f"❌ Failed to connect to {port}: {e}")
                    continue
        
        print(f"❌ No LiDAR Arduino found on any port")
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
        
        self.running = True
        
        # Start reader thread
        arduino_thread = threading.Thread(target=self.run_arduino_reader, daemon=True)
        arduino_thread.start()
        
        print("✅ LiDAR Detection System started")
        return True
    
    def stop(self):
        """Stop the LiDAR detection system."""
        print("🛑 Stopping LiDAR Detection System...")
        self.running = False
        
        if self.arduino:
            self.arduino.close()
        
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
