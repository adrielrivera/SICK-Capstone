#!/usr/bin/env python3
"""
Quick test script to verify GPIO 18 output works
"""
import RPi.GPIO as GPIO
import time

GPIO_PIN = 18  # GPIO18 (Physical Pin 12)

try:
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(GPIO_PIN, GPIO.OUT)
    
    print(f"Testing GPIO {GPIO_PIN}...")
    print("Setting HIGH (3.3V) for 2 seconds...")
    GPIO.output(GPIO_PIN, GPIO.HIGH)
    time.sleep(2)
    
    print("Setting LOW (0V) for 2 seconds...")
    GPIO.output(GPIO_PIN, GPIO.LOW)
    time.sleep(2)
    
    print("Setting HIGH again for 2 seconds...")
    GPIO.output(GPIO_PIN, GPIO.HIGH)
    time.sleep(2)
    
    print("Setting LOW (final)...")
    GPIO.output(GPIO_PIN, GPIO.LOW)
    
    print("✅ GPIO test complete! Check with oscilloscope.")
    
except Exception as e:
    print(f"❌ Error: {e}")
    print("Try running with: sudo python3 test_gpio18.py")
finally:
    GPIO.cleanup()

