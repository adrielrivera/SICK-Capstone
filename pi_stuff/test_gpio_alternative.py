#!/usr/bin/env python3
"""
Test alternative GPIO pins to verify hardware works
"""
import RPi.GPIO as GPIO
import time
import sys

# Try multiple GPIO pins
TEST_PINS = [18, 17, 27, 22, 23]  # Common GPIO pins

print("=" * 60)
print("Testing Multiple GPIO Pins")
print("=" * 60)

GPIO.setmode(GPIO.BCM)

for pin in TEST_PINS:
    try:
        print(f"\nTesting GPIO {pin}...")
        GPIO.setup(pin, GPIO.OUT, initial=GPIO.LOW)
        
        print(f"  Setting HIGH...")
        GPIO.output(pin, GPIO.HIGH)
        time.sleep(0.1)
        state = GPIO.input(pin)
        print(f"  Read back: {state} ({'HIGH' if state else 'LOW'})")
        
        if state == GPIO.HIGH:
            print(f"  ✅ GPIO {pin} works!")
            time.sleep(1)
            GPIO.output(pin, GPIO.LOW)
            break
        else:
            print(f"  ❌ GPIO {pin} not responding")
            GPIO.cleanup()
            GPIO.setmode(GPIO.BCM)
    except Exception as e:
        print(f"  ❌ Error with GPIO {pin}: {e}")
        try:
            GPIO.cleanup()
            GPIO.setmode(GPIO.BCM)
        except:
            pass

GPIO.cleanup()
print("\nTest complete!")

