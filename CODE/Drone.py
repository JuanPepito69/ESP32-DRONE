from inputs import get_gamepad
import serial
import time

# Config
ser = serial.Serial('COM3', 115200, write_timeout=0, timeout=0.001)
ser.reset_input_buffer()

throttle = yaw = pitch = roll = 0
armed = False
DEADZONE = 5

def apply_deadzone(value, threshold=DEADZONE):
    if abs(value) < threshold:
        return 0
    if value > 0:
        return int((value - threshold) * 127 / (127 - threshold))
    else:
        return int((value + threshold) * 127 / (127 - threshold))

print("Manette Xbox connectée!")
print("A = ARM | B = DISARM")
print("Sticks: Gauche=Throttle/Yaw | Droit=Pitch/Roll\n")

# Stats (optionnel)
packets_sent = 0
start_time = time.time()

try:
    while True:
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
                print("ARMED")
            elif event.code == 'BTN_EAST' and event.state == 1:
                armed = False
                throttle = 0  # Sécurité
                print("DISARMED")
        
        arm_val = 1 if armed else 0
        data = f"{throttle},{yaw},{pitch},{roll},{arm_val}\n"
        ser.write(data.encode())
        packets_sent += 1
        
        # Affichage toutes les secondes
        if packets_sent % 100 == 0:
            hz = packets_sent / (time.time() - start_time)
            print(f"T:{throttle:4} Y:{yaw:4} P:{pitch:4} R:{roll:4} | {hz:.0f}Hz", end='\r')

except KeyboardInterrupt:
    print("\n\nArrêt ground station")
    armed = False
    data = f"0,0,0,0,0\n"
    ser.write(data.encode())
    ser.close()