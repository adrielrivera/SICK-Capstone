# SICK7 Unified System - Credit Tracking + Safety System

## Overview

This unified Arduino system integrates:
- **Credit Tracking**: Detects credit insertion and tracks remaining strikes
- **PBT Processing**: Reads PBT sensor and sends data to Pi
- **GPIO Control**: Controls arcade motherboard pins for scoring
- **Safety Alarm**: Controls buzzer system based on credit availability

## Key Features

### Credit System
- **1 credit = 2 strikes** (PBT hits)
- **Falling edge detection** on Pin 2 for credit insertion
- **Automatic tracking** of remaining strikes
- **Safety system activation** only when credits available

### Safety System
- **Alarm only active** when credits are available
- **LiDAR trigger integration** via Pi communication
- **Automatic deactivation** when strikes exhausted
- **5-second siren** with sweeping tone

### PBT Processing
- **800 samples/sec** PBT sensor reading
- **Auto-calibration** on startup
- **GPIO control** for arcade scoring
- **Real-time diagnostics**

## Hardware Connections

### Arduino Pins
```
PBT Sensor:     A0
Credit Signal:  Pin 2 (with pull-up resistor)
LiDAR Trigger:  Pin 8
Buzzer:         Pin 9
LED:            Pin 13
Arcade GPIO:    Pins 5, 6
```

### Credit Signal
- **Falling edge** triggers credit insertion
- **Pull-up resistor** required on Pin 2
- **Multiple credits** supported

### LiDAR Integration
- **Pi sends LIDAR_TRIGGER** command
- **Arduino checks credit status** before activating alarm
- **Safety system must be active** for alarm to sound

## Communication Protocol

### Pi → Arduino Commands
```
PBT_HIT          - PBT hit detected (decrements strikes)
LIDAR_TRIGGER    - Safety violation detected
PIN6_HIGH        - Arcade Pin 6 HIGH
PIN6_LOW         - Arcade Pin 6 LOW
PIN5_HIGH        - Arcade Pin 5 HIGH
PIN5_LOW         - Arcade Pin 5 LOW
RESET_GPIO       - Reset GPIO to idle state
STATUS           - Get system status
```

### Arduino → Pi Messages
```
# CREDIT INSERTED: Total=X Strikes=Y Safety=ACTIVE/INACTIVE
# PBT HIT: Total=X Strikes=Y Safety=ACTIVE/INACTIVE
# LiDAR TRIGGER - Siren ON (Safety System Active)
# LiDAR TRIGGER IGNORED - No credits remaining
# GAME OVER - No strikes remaining
```

## System Logic

### Credit Tracking
1. **Credit inserted** → `creditCount++`
2. **PBT hit detected** → `pbtHitCount++`
3. **Remaining strikes** = `(credits × 2) - pbt_hits`
4. **Safety system active** = `remainingStrikes > 0`

### Safety System
1. **LiDAR detects violation** → Pi sends `LIDAR_TRIGGER`
2. **Arduino checks credits** → Only activate if `safetySystemActive`
3. **Alarm sounds** → 5-second siren with LED blinking
4. **Game over** → No more strikes, safety system deactivated

### PBT Processing
1. **Sensor reading** → 800 samples/sec with averaging
2. **Peak detection** → Pi processes envelope and triggers
3. **GPIO control** → Arduino controls arcade pins
4. **Credit tracking** → Pi sends `PBT_HIT` to Arduino

## Diagnostics

### Status Messages
```
# STATUS: Credits=2 PBT=3 Strikes=1 Safety=ACTIVE
# STATS: samples=4000 raw_min=350 raw_max=450 range=100 (0.49V)
# CREDITS: Total=2 PBT=3 Strikes=1 Safety=ACTIVE
```

### LED Indicators
- **Blinking during activity** - PBT sensor activity
- **Solid during siren** - Safety alarm active
- **Off when idle** - No activity or credits

## Integration with Pi Systems

### PBT System (app_combined.py)
- Sends `PBT_HIT` when peak detected
- Controls GPIO for arcade scoring
- Maintains existing PBT processing logic

### LiDAR System (tim240_run.py)
- Sends `LIDAR_TRIGGER` when person detected
- Arduino checks credit status before alarm
- Integrated with kite-shaped safety field

## Safety Features

### Credit-Based Activation
- **Alarm only sounds** when credits available
- **Automatic deactivation** when strikes exhausted
- **Clear game state** tracking

### Robust Error Handling
- **Serial communication** error recovery
- **GPIO state** management
- **Diagnostic output** for troubleshooting

## Usage

1. **Upload code** to Arduino
2. **Connect hardware** according to pin assignments
3. **Start Pi systems** (PBT and LiDAR)
4. **Insert credits** to activate safety system
5. **Play game** - strikes automatically tracked
6. **Safety system** deactivates when game over

## Troubleshooting

### No Credit Detection
- Check Pin 2 pull-up resistor
- Verify falling edge signal
- Monitor serial output for credit messages

### Alarm Not Working
- Check credit status with `STATUS` command
- Verify LiDAR trigger messages
- Ensure safety system is active

### PBT Issues
- Check A0 connection
- Monitor baseline calibration
- Verify GPIO commands from Pi

This unified system provides robust credit tracking and safety control for the SICK7 arcade machine!
