# DSHOT300 Test Implementation for ESP32-S3 Drone

## Overview

This proof-of-concept demonstrates DSHOT300 digital ESC protocol implementation on the ESP32-S3 using the RMT (Remote Control Transceiver) peripheral. DSHOT is a superior alternative to traditional PWM for ESC control.

## Why DSHOT300 vs PWM?

### Advantages of DSHOT300:

1. **Digital Signal** - More resistant to noise and interference
2. **No Calibration** - ESCs don't need throttle range calibration
3. **Faster Response** - Lower latency than PWM (250Hz+ vs 50-490Hz PWM)
4. **Bidirectional Communication** - Can request telemetry from ESCs
5. **CRC Error Checking** - Detects corrupted frames
6. **Special Commands** - Motor configuration, beeps, direction control
7. **Precise Control** - 2048 throttle steps vs ~1000 in PWM

### DSHOT Variants:
- **DSHOT150** - 150kbit/s (older ESCs)
- **DSHOT300** - 300kbit/s (good balance, widely supported) ⭐ This implementation
- **DSHOT600** - 600kbit/s (high performance)
- **DSHOT1200** - 1200kbit/s (bleeding edge)

## Hardware Requirements

- **ESP32-S3** (this implementation is optimized for S3)
- **DSHOT-compatible ESCs** (most modern BLHeli_S, BLHeli_32, AM32)
- **4 motors** connected to GPIOs 1, 2, 42, 41

### Motor Configuration:
```
     FRONT
  3(CCW) 1(CW)
     \ /
     / \
  2(CW) 4(CCW)
    BACK
```

## Protocol Details

### DSHOT300 Timing:
- **Bit rate**: 300 kbit/s (3.33µs per bit)
- **Logic 1**: 2.5µs HIGH, 0.83µs LOW (75% duty cycle)
- **Logic 0**: 1.25µs HIGH, 2.08µs LOW (37.5% duty cycle)

### Frame Structure (16 bits):
```
[11 bits throttle] [1 bit telemetry] [4 bits CRC]
```

### Throttle Values:
- **0-47**: Special commands
- **48**: Minimum throttle (motors spin)
- **1000**: ~50% throttle
- **2047**: Maximum throttle

### Special Commands (0-47):
| Command | Description |
|---------|-------------|
| 0 | Motor stop |
| 1-5 | Beep patterns |
| 6 | ESC info request |
| 7-8 | Spin direction |
| 9 | 3D mode OFF |
| 10 | 3D mode ON |
| 11 | Settings request |
| 12 | Save settings to EEPROM |
| 20 | Normal spin direction |
| 21 | Reversed spin direction |

## Installation & Usage

### 1. Upload the Sketch
```bash
# Open in Arduino IDE
arduino --upload dshot300.ino

# Or use PlatformIO
pio run -t upload
```

### 2. Open Serial Monitor
- Baud rate: **115200**
- Line ending: **Newline**

### 3. Safety First
⚠️ **REMOVE ALL PROPELLERS BEFORE TESTING!** ⚠️

### 4. Arm the ESCs
```
Type: a
```
You should hear confirmation beeps from the ESCs.

### 5. Test Commands

#### Set All Motors to Same Throttle:
```
# Stop motors
0

# Minimum throttle (48)
48

# Half throttle (~1000)
1000

# Maximum throttle (2047)
2047
```

#### Control Individual Motors:
```
# Motor 1 to 500
m1 500

# Motor 2 to 1000
m2 1000

# Motor 3 to 200
m3 200

# Motor 4 to 0 (stop)
m4 0
```

#### Run Tests:
```
# Beep all motors
b

# Throttle ramp test (0 → 500 → 0)
r

# Show current status
info

# Show help menu
help
```

#### Special DSHOT Commands:
```
# Beep pattern 1
cmd 1

# Reverse motor direction
cmd 21

# Normal motor direction
cmd 20

# Save settings
cmd 12
```

## RMT Configuration

The implementation uses ESP32's RMT peripheral:

```cpp
- Clock: 80MHz APB / 4 = 20MHz (0.05µs per tick)
- Channels: 0-3 (one per motor)
- Mode: TX only, no carrier
- Memory: 1 block per channel
```

### Timing in RMT Ticks:
```cpp
Logic 1 HIGH: 50 ticks (2.5µs)
Logic 1 LOW:  17 ticks (0.85µs)
Logic 0 HIGH: 25 ticks (1.25µs)
Logic 0 LOW:  42 ticks (2.1µs)
```

## Code Structure

### Key Functions:

- `initDShot()` - Initialize RMT channels
- `encodeDShotPacket()` - Encode 16-bit DSHOT frame with CRC
- `sendDShotValue()` - Send throttle value to motor
- `sendDShotCommand()` - Send special command to motor
- `calculateCRC()` - Compute CRC4 checksum
- `armESCs()` - ESC arming sequence
- `loop()` - Continuous signal transmission at ~250Hz

### Main Loop Behavior:
The loop continuously sends DSHOT frames at ~250Hz (every 4ms) to maintain signal. This is required by the DSHOT protocol - unlike PWM which can hold a value, DSHOT requires constant updates.

## Integration with Main Drone Code

To integrate DSHOT300 into your main drone code (`Recept.ino`):

### 1. Replace ESP32Servo with DSHOT:
```cpp
// Remove:
#include <ESP32Servo.h>
Servo esc[4];

// Add:
#include <driver/rmt.h>
// Copy DSHOT functions from dshot300.ino
```

### 2. Update ESC Initialization:
```cpp
// Replace:
esc[i].attach(escPins[i], 1000, 2000);
esc[i].writeMicroseconds(1000);

// With:
initDShot();
sendAllMotors(0);  // Arm with 0 throttle
```

### 3. Update Motor Commands:
```cpp
// Replace PWM microseconds (1000-2000):
esc[i].writeMicroseconds(1500);

// With DSHOT values (0-2047):
// Convert: PWM 1000-2000µs → DSHOT 48-2047
int dshotValue = map(pwmValue, 1000, 2000, 48, 2047);
sendDShotValue(motorIndex, dshotValue);
```

### 4. Update mixAndApply() Function:
```cpp
void mixAndApply(Command* c) {
  // ... existing mixing logic ...

  // Convert PWM range to DSHOT range
  m1 = map(m1, THROTTLE_ARM, THROTTLE_MAX, 0, 2047);
  m2 = map(m2, THROTTLE_ARM, THROTTLE_MAX, 0, 2047);
  m3 = map(m3, THROTTLE_ARM, THROTTLE_MAX, 0, 2047);
  m4 = map(m4, THROTTLE_ARM, THROTTLE_MAX, 0, 2047);

  // Send DSHOT commands
  sendDShotValue(0, m1);
  sendDShotValue(1, m2);
  sendDShotValue(2, m3);
  sendDShotValue(3, m4);
}
```

## Troubleshooting

### Motors Don't Arm
- Check ESC supports DSHOT300 (BLHeli_S, BLHeli_32, AM32)
- Verify correct GPIO pins
- Try sending zero throttle longer (increase loop in `armESCs()`)
- Some ESCs need DSHOT command 0 first

### Motors Don't Respond to Throttle
- Ensure ESCs are armed (should beep)
- Check throttle range (48-2047, not 0-47)
- Verify RMT timing is correct for your ESC
- Try DSHOT600 if 300 doesn't work (change timing constants)

### Erratic Motor Behavior
- Check power supply is adequate
- Verify signal ground is connected
- Ensure no electromagnetic interference
- Check CRC calculation is correct

### One or More Motors Don't Work
- Verify GPIO pin assignments
- Check RMT channel isn't used elsewhere
- Test ESC with PWM to verify hardware
- Check motor/ESC connections

## Performance Notes

- **Update Rate**: 250Hz (4ms interval) - can be increased to 1-2kHz
- **Latency**: ~1-4ms (vs 2-20ms for PWM)
- **CPU Usage**: Minimal - RMT is hardware-driven
- **Memory**: 64 bytes per motor (4 × 16 RMT items)

## Future Enhancements

1. **Bidirectional DSHOT** - Read telemetry (RPM, current, temperature)
2. **DSHOT600/1200** - Higher speed variants
3. **Dynamic Throttle Mapping** - Optimize for flight characteristics
4. **ESC Configuration** - Use commands to configure ESCs
5. **Error Detection** - Monitor and log CRC errors

## References

- [DSHOT Protocol Specification](https://github.com/betaflight/betaflight/wiki/DSHOT-ESC-Protocol)
- [ESP32 RMT Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/rmt.html)
- [BLHeli_32 Manual](https://github.com/bitdump/BLHeli/tree/master/BLHeli_32%20ARM)

## License

This is a proof-of-concept for educational purposes. Modify and use as needed for your drone project.

---

**Happy Flying! 🚁**

Remember: Safety first - always test without propellers!
