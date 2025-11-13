#!/usr/bin/env python3
"""
Quick test script to verify GPIO 18 output works
"""
import RPi.GPIO as GPIO
import time
import sys

GPIO_PIN = 18  # GPIO18 (Physical Pin 12)

print("=" * 60)
print("GPIO 18 Test Script")
print("=" * 60)
print(f"Target: GPIO {GPIO_PIN} (BCM) = Physical Pin 12")
print()

try:
    # Check if GPIO is already in use
    print("Step 1: Setting GPIO mode to BCM...")
    GPIO.setmode(GPIO.BCM)
    print("   ✅ GPIO mode set")
    
    print(f"Step 2: Setting GPIO {GPIO_PIN} as OUTPUT...")
    GPIO.setup(GPIO_PIN, GPIO.OUT, initial=GPIO.LOW)
    print("   ✅ GPIO configured as OUTPUT")
    
    # Read initial state
    initial_state = GPIO.input(GPIO_PIN)
    print(f"   Initial state: {initial_state} ({'HIGH' if initial_state else 'LOW'})")
    
    print()
    print("Step 3: Testing GPIO output...")
    print("   Setting HIGH (should be 3.3V)...")
    GPIO.output(GPIO_PIN, GPIO.HIGH)
    time.sleep(0.1)  # Small delay for signal to settle
    high_state = GPIO.input(GPIO_PIN)
    print(f"   Read back: {high_state} ({'HIGH' if high_state else 'LOW'})")
    if high_state != GPIO.HIGH:
        print("   ⚠️  WARNING: GPIO read back LOW when set to HIGH!")
    time.sleep(2)
    
    print("   Setting LOW (should be 0V)...")
    GPIO.output(GPIO_PIN, GPIO.LOW)
    time.sleep(0.1)
    low_state = GPIO.input(GPIO_PIN)
    print(f"   Read back: {low_state} ({'HIGH' if low_state else 'LOW'})")
    if low_state != GPIO.LOW:
        print("   ⚠️  WARNING: GPIO read back HIGH when set to LOW!")
    time.sleep(2)
    
    print("   Setting HIGH again...")
    GPIO.output(GPIO_PIN, GPIO.HIGH)
    time.sleep(0.1)
    high_state2 = GPIO.input(GPIO_PIN)
    print(f"   Read back: {high_state2} ({'HIGH' if high_state2 else 'LOW'})")
    time.sleep(2)
    
    print("   Setting LOW (final)...")
    GPIO.output(GPIO_PIN, GPIO.LOW)
    time.sleep(0.1)
    low_state2 = GPIO.input(GPIO_PIN)
    print(f"   Read back: {low_state2} ({'HIGH' if low_state2 else 'LOW'})")
    
    print()
    print("=" * 60)
    if high_state == GPIO.HIGH and low_state == GPIO.LOW:
        print("✅ GPIO test PASSED - GPIO responds correctly")
        print("   If oscilloscope shows no change, check:")
        print("   1. Physical connection to GPIO 18 (Pin 12)")
        print("   2. Ground connection")
        print("   3. Oscilloscope probe settings")
    else:
        print("❌ GPIO test FAILED - GPIO not responding correctly")
        print("   Possible issues:")
        print("   1. GPIO pin conflict (another process using it)")
        print("   2. Hardware issue")
        print("   3. Wrong pin number")
    print("=" * 60)
    
except RuntimeError as e:
    print(f"❌ RuntimeError: {e}")
    print()
    print("This usually means:")
    print("  1. Need to run with sudo: sudo python3 test_gpio18.py")
    print("  2. Or add user to gpio group: sudo usermod -a -G gpio $USER")
    print("  3. Then log out and log back in")
    sys.exit(1)
except PermissionError as e:
    print(f"❌ PermissionError: {e}")
    print()
    print("Run with sudo: sudo python3 test_gpio18.py")
    sys.exit(1)
except Exception as e:
    print(f"❌ Error: {type(e).__name__}: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)
finally:
    try:
        GPIO.cleanup()
        print("\nGPIO cleaned up")
    except:
        pass

