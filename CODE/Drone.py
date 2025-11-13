"""
Optimized Xbox Controller to ESP32 NRF24 Ground Station
- Binary protocol for faster transmission
- Rate limiting to prevent serial buffer overflow
- Efficient input processing
- Reduced CPU usage
"""

from inputs import get_gamepad
import serial
import struct
import time

# ===== CONFIGURATION =====
SERIAL_PORT = 'COM3'
BAUD_RATE = 115200
TARGET_UPDATE_RATE = 100  # Hz (100Hz = 10ms between packets)
DEADZONE = 5

# ===== SERIAL SETUP =====
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, write_timeout=0.01, timeout=0)
    ser.reset_input_buffer()
    ser.reset_output_buffer()
except Exception as e:
    print(f"Error opening serial port: {e}")
    exit(1)

# ===== STATE VARIABLES =====
throttle = yaw = pitch = roll = 0
armed = False
last_send_time = 0
UPDATE_INTERVAL = 1.0 / TARGET_UPDATE_RATE

def apply_deadzone(value, threshold=DEADZONE):
    """Apply deadzone and scale to -127 to 127 range"""
    if abs(value) < threshold:
        return 0
    if value > 0:
        return int((value - threshold) * 127 / (127 - threshold))
    else:
        return int((value + threshold) * 127 / (127 - threshold))

def send_command():
    """Send command using optimized binary protocol"""
    global packets_sent, last_send_time

    # Pack data into binary format (5 signed bytes)
    # Format: 5 signed chars (int8) for throttle, yaw, pitch, roll, armed
    try:
        arm_val = 1 if armed else 0
        data = struct.pack('bbbbb', throttle, yaw, pitch, roll, arm_val)
        ser.write(data)
        packets_sent += 1
        last_send_time = time.time()
    except serial.SerialException as e:
        print(f"\nSerial error: {e}")
    except Exception as e:
        print(f"\nUnexpected error: {e}")

print("=" * 50)
print("Optimized Xbox Controller Ground Station")
print("=" * 50)
print("Controls:")
print("  A Button = ARM")
print("  B Button = DISARM")
print("  Left Stick  = Throttle (Y) / Yaw (X)")
print("  Right Stick = Pitch (Y) / Roll (X)")
print(f"\nUpdate Rate: {TARGET_UPDATE_RATE}Hz")
print("=" * 50)
print()

# ===== STATS =====
packets_sent = 0
start_time = time.time()
last_print_time = start_time

try:
    while True:
        current_time = time.time()

        # Process all pending gamepad events (non-blocking)
        try:
            events = get_gamepad()
            for event in events:
                if event.code == 'ABS_Y':
                    throttle = apply_deadzone(-int(event.state / 257))
                elif event.code == 'ABS_X':
                    yaw = apply_deadzone(int(event.state / 257))
                elif event.code == 'ABS_RY':
                    pitch = apply_deadzone(-int(event.state / 257))
                elif event.code == 'ABS_RX':
                    roll = apply_deadzone(int(event.state / 257))
                elif event.code == 'BTN_SOUTH' and event.state == 1:
                    armed = True
                    print("✓ ARMED                                    ")
                elif event.code == 'BTN_EAST' and event.state == 1:
                    armed = False
                    throttle = 0  # Safety: zero throttle on disarm
                    print("✗ DISARMED                                 ")
        except Exception as e:
            # Handle gamepad disconnection gracefully
            print(f"\nGamepad error: {e}")
            time.sleep(0.1)
            continue

        # Rate-limited transmission
        if current_time - last_send_time >= UPDATE_INTERVAL:
            send_command()

        # Display stats every 0.5 seconds
        if current_time - last_print_time >= 0.5:
            elapsed = current_time - start_time
            hz = packets_sent / elapsed if elapsed > 0 else 0
            print(f"T:{throttle:4} Y:{yaw:4} P:{pitch:4} R:{roll:4} | "
                  f"Rate:{hz:.1f}Hz Pkts:{packets_sent}", end='\r')
            last_print_time = current_time

        # Small sleep to prevent CPU spinning (will still achieve target rate)
        time.sleep(0.001)  # 1ms sleep

except KeyboardInterrupt:
    print("\n\n" + "=" * 50)
    print("Shutting down ground station...")
    print("=" * 50)

    # Send disarm command
    armed = False
    throttle = yaw = pitch = roll = 0
    send_command()
    time.sleep(0.1)  # Ensure command is sent

    ser.close()
    elapsed = time.time() - start_time
    print(f"Session duration: {elapsed:.1f}s")
    print(f"Total packets: {packets_sent}")
    print(f"Average rate: {packets_sent/elapsed:.1f}Hz")
    print("Safe shutdown complete.")