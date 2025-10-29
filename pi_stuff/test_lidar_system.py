#!/usr/bin/env python3
"""
Test script for LiDAR Detection System
Tests Arduino communication without webapp
"""

import time
from lidar_detection_reader import LiDARDetectionSystem

def status_callback(status):
    print(f"Status update: {status}")

def main():
    print("=" * 50)
    print("LiDAR Detection System Test")
    print("=" * 50)
    
    system = LiDARDetectionSystem()
    system.add_status_callback(status_callback)
    
    try:
        if system.start():
            print("✅ System started successfully")
            print("Monitoring Arduino for 30 seconds...")
            print("Press Ctrl+C to stop early")
            
            time.sleep(30)
        else:
            print("❌ Failed to start system")
    except KeyboardInterrupt:
        print("\nStopping test...")
    finally:
        system.stop()
        print("Test completed")

if __name__ == "__main__":
    main()
