/*
 * ESP32-S3 Drone Flight Controller
 *
 * CASCADED PID CONTROL SYSTEM:
 * This implementation uses a dual-loop (cascaded) PID control architecture:
 *
 * 1. OUTER LOOP - Position/Angle PID (PID_Position):
 *    - Input: Desired angles (roll/pitch in degrees)
 *    - Output: Desired angular rates (deg/s)
 *    - Provides stable angle hold and smooth flight characteristics
 *
 * 2. INNER LOOP - Rate/Gyro PID (PID_Gyro):
 *    - Input: Desired angular rates (from position PID or direct rate commands)
 *    - Output: Motor PWM corrections
 *    - Provides fast response and stability
 *
 * OPTIMIZATIONS FOR ESP32-S3:
 * - Inline helper functions for fast math operations
 * - Minimal floating-point operations
 * - Anti-windup on integral terms
 * - Proper derivative calculation using error difference
 * - Efficient time delta calculation
 *
 * Control flow: Command -> Position PID -> Rate PID -> Motors
 */

#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <Wire.h>

Adafruit_ICM20948 icm;
Madgwick filter;

#define SDA_PIN 38
#define SCL_PIN 39

RF24 radio(4, 5); // CE, CSN (ESP32-S3)
const byte address[6] = "DRONE";

struct Command {
  int16_t throttle;
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  uint8_t armed;
  uint8_t checksum;
} cmd, lastValidCmd;

Servo esc[4];
const int escPins[4] = {1, 2, 42, 41};

float roll = 0;
float pitch = 0;
float yaw = 0;
float altitude = 0;
const float sens = 5; 

// Constantes moteurs
const int THROTTLE_MIN = 1020;  // Démarrage
const int THROTTLE_ARM = 1000;  // ARM
const int THROTTLE_MAX = 2000;

// Constantes PID ---------------------------------

// RATE PID (Gyro) - Inner loop
// ROLL Rate
const float KpR = 1.5;
const float KiR = 0.3;
const float KdR = 0.05;
float IErrRoll = 0;
float prevErrRoll = 0;

// PITCH Rate
const float KpP = 1.5;
const float KiP = 0.3;
const float KdP = 0.05;
float IErrPitch = 0;
float prevErrPitch = 0;

// YAW Rate
const float KpY = 2.0;
const float KiY = 0.5;
const float KdY = 0.0;
float IErrYaw = 0;
float prevErrYaw = 0;

// Integral limits (anti-windup)
const float I_LIMIT = 400.0;

// POSITION PID (Angle) - Outer loop
// ROLL Angle
const float KpR_pos = 2.0;
const float KiR_pos = 0.0;
const float KdR_pos = 0.0;
float IErrRoll_pos = 0;
float prevErrRoll_pos = 0;

// PITCH Angle
const float KpP_pos = 2.0;
const float KiP_pos = 0.0;
const float KdP_pos = 0.0;
float IErrPitch_pos = 0;
float prevErrPitch_pos = 0;

// Max angular rate output from position PID (deg/s)
const float MAX_RATE_OUTPUT = 200.0;

// ------------------------------------------------

// Gyro and attitude state variables
float gyroRoll = 0;   // deg/s
float gyroPitch = 0;  // deg/s
float gyroYaw = 0;    // deg/s
float angleRoll = 0;  // degrees
float anglePitch = 0; // degrees

float lastUpdate = 0;
unsigned long lastRX = 0;
bool armed = false;

// ===== HELPER FUNCTIONS =====

// Fast float mapping (optimized for ESP32)
inline float mapf(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Fast float constrain
inline float constrain_float(float value, float min_val, float max_val) {
  if (value < min_val) return min_val;
  if (value > max_val) return max_val;
  return value;
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  while (!icm.begin_I2C(0x68, &Wire)) {
    Serial.println("ICM20948 non trouvé!");
    delay(50);
  }
  icm.setAccelRateDivisor(0);  // Max rate
  icm.setGyroRateDivisor(0);   // Max rate
  filter.begin(1000); // 1000 Hz estimé
  
  Serial.println("Madgwick initialisé");
  Serial.println("\nAttitude (degrés):");
  
  
  // Init SPI + nRF24
  SPI.begin(12, 13, 11);
  radio.begin();
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(108);
  radio.setAutoAck(false);
  radio.setPayloadSize(sizeof(cmd));
  radio.openReadingPipe(1, address);
  radio.startListening();
  
  // Init ESC

  for(int i = 0; i < 4; i++) {
    esc[i].attach(escPins[i], 1000, 2000);
    esc[i].writeMicroseconds(THROTTLE_ARM);
  }
  
  Serial.println("Drone prêt - Attente commandes...");
  delay(2000);
  lastUpdate = micros();
}

uint8_t calcChecksum(Command* c) {
  uint8_t sum = 0;
  uint8_t* ptr = (uint8_t*)c;
  for(int i = 0; i < sizeof(Command) - 1; i++) {
    sum ^= ptr[i];
  }
  return sum;
}

void failsafe() {
  armed = false;
  for(int i = 0; i < 4; i++) {
    esc[i].writeMicroseconds(THROTTLE_ARM);
  }
  Serial.println("⚠ FAILSAFE !");
}

void mixAndApply(Command* c) {
  // Conversion -127/127 → throttle PWM
  int baseThrottle = map(c->throttle, -127, 127, THROTTLE_ARM, THROTTLE_MAX);
  baseThrottle = constrain(baseThrottle, THROTTLE_ARM, THROTTLE_MAX);
  
  // Mixer basique (sans stabilisation pour l'instant)
  // Configuration quadcopter X:
  //     AVANT
  //  3(CCW) 1(CW)
  //     \ /
  //     / \
  //  2(CW) 4(CCW)
  
  // Yaw: rotation (-127 à 127)
  // Pitch: avant/arrière (-127 à 127)
  // Roll: gauche/droite (-127 à 127)
  
  int yawEffect = map(c->yaw, -127, 127, -100, 100);
  int pitchEffect = map(c->pitch, -127, 127, -100, 100);
  int rollEffect = map(c->roll, -127, 127, -100, 100);
  
  // Moteur 1 (avant-droit, CW)
  int m1 = baseThrottle - rollEffect + pitchEffect - yawEffect;
  
  // Moteur 2 (arrière-gauche, CW)
  int m2 = baseThrottle + rollEffect - pitchEffect - yawEffect;
  
  // Moteur 3 (avant-gauche, CCW)
  int m3 = baseThrottle + rollEffect + pitchEffect + yawEffect;
  
  // Moteur 4 (arrière-droit, CCW)
  int m4 = baseThrottle - rollEffect - pitchEffect + yawEffect;
  
  // Constrain
  m1 = constrain(m1, THROTTLE_ARM, THROTTLE_MAX);
  m2 = constrain(m2, THROTTLE_ARM, THROTTLE_MAX);
  m3 = constrain(m3, THROTTLE_ARM, THROTTLE_MAX);
  m4 = constrain(m4, THROTTLE_ARM, THROTTLE_MAX);
  
  // Apply
  if (armed && baseThrottle > THROTTLE_MIN) {
    esc[0].writeMicroseconds(m1);
    esc[1].writeMicroseconds(m2);
    esc[2].writeMicroseconds(m3);
    esc[3].writeMicroseconds(m4);
  } else {
    // Désarmé ou throttle bas = moteurs OFF
    for(int i = 0; i < 4; i++) {
      esc[i].writeMicroseconds(THROTTLE_ARM);
    }
  }
}

void loop() {
  // Calculate dt for PID (in seconds)
  unsigned long currentTime = micros();
  float dt = (currentTime - lastUpdate) / 1000000.0; // Convert to seconds
  lastUpdate = currentTime;

  // Constrain dt to reasonable values (handle first loop and anomalies)
  if (dt > 0.1 || dt <= 0) dt = 0.01; // Default 10ms if invalid

  // Update IMU readings
  updateIMU();

  // Réception commandes
  if (radio.available()) {
    radio.read(&cmd, sizeof(cmd));

    // Vérif checksum
    if (calcChecksum(&cmd) == cmd.checksum) {
      lastValidCmd = cmd;
      lastRX = millis();

      // Gestion ARM/DISARM
      if (cmd.armed && !armed) {
        armed = true;
        Serial.println("✓ ARMED");
      } else if (!cmd.armed && armed) {
        armed = false;
        Serial.println("DISARMED");
      }
    } else {
      Serial.println("Checksum error");
    }
  }

  // PID Control System (cascaded)
  if (armed) {
    // Base throttle from command
    int baseThrottle = map(lastValidCmd.throttle, -127, 127, THROTTLE_ARM, THROTTLE_MAX);
    baseThrottle = constrain(baseThrottle, THROTTLE_ARM, THROTTLE_MAX);

    // Convert commands to angle targets (degrees) for position control
    // Range: -45 to +45 degrees for roll/pitch is typical for drones
    float angleRollCmd = mapf(lastValidCmd.roll, -127, 127, -45.0, 45.0);
    float anglePitchCmd = mapf(lastValidCmd.pitch, -127, 127, -45.0, 45.0);

    // Yaw is rate-controlled directly (deg/s)
    float rateYawCmd = mapf(lastValidCmd.yaw, -127, 127, -180.0, 180.0);

    // Position PID: converts angle commands to rate commands
    float rateRollCmd, ratePitchCmd;
    PID_Position(angleRollCmd, anglePitchCmd, rateRollCmd, ratePitchCmd, dt);

    // Rate PID: converts rate commands to motor outputs
    PID_Gyro(rateRollCmd, ratePitchCmd, rateYawCmd, baseThrottle, dt);
  } else {
    // Not armed - ensure motors are off
    for(int i = 0; i < 4; i++) {
      esc[i].writeMicroseconds(THROTTLE_ARM);
    }
  }

  // Failsafe si pas de RX depuis 5000ms
  if (millis() - lastRX > 5000 && armed) {
    failsafe();
  }
}

// PID Rate Controller (Gyro-based) - OPTIMIZED for ESP32S3
// Takes desired angular rates and outputs motor corrections
// Inputs: rate commands (deg/s) for roll, pitch, yaw
// Outputs: motor PWM values via ESC array
void PID_Gyro(float rateRollCmd, float ratePitchCmd, float rateYawCmd, int baseThrottle, float dt) {
  // Calculate errors (setpoint - measured)
  float errRoll = rateRollCmd - gyroRoll;
  float errPitch = ratePitchCmd - gyroPitch;
  float errYaw = rateYawCmd - gyroYaw;

  // ROLL PID calculation
  // Integral with anti-windup
  IErrRoll += errRoll * dt;
  IErrRoll = constrain_float(IErrRoll, -I_LIMIT, I_LIMIT);
  // Derivative (using error derivative for noise reduction)
  float DErrRoll = (errRoll - prevErrRoll) / dt;
  prevErrRoll = errRoll;
  // PID output
  float rollEffect = KpR * errRoll + KiR * IErrRoll + KdR * DErrRoll;

  // PITCH PID calculation
  IErrPitch += errPitch * dt;
  IErrPitch = constrain_float(IErrPitch, -I_LIMIT, I_LIMIT);
  float DErrPitch = (errPitch - prevErrPitch) / dt;
  prevErrPitch = errPitch;
  float pitchEffect = KpP * errPitch + KiP * IErrPitch + KdP * DErrPitch;

  // YAW PID calculation
  IErrYaw += errYaw * dt;
  IErrYaw = constrain_float(IErrYaw, -I_LIMIT, I_LIMIT);
  float DErrYaw = (errYaw - prevErrYaw) / dt;
  prevErrYaw = errYaw;
  float yawEffect = KpY * errYaw + KiY * IErrYaw + KdY * DErrYaw;

  // Motor mixing for X configuration
  // Configuration quadcopter X:
  //     AVANT
  //  3(CCW) 1(CW)
  //     \ /
  //     / \
  //  2(CW) 4(CCW)

  int m1 = baseThrottle - rollEffect + pitchEffect - yawEffect;
  int m2 = baseThrottle + rollEffect - pitchEffect - yawEffect;
  int m3 = baseThrottle + rollEffect + pitchEffect + yawEffect;
  int m4 = baseThrottle - rollEffect - pitchEffect + yawEffect;

  // Constrain and apply
  m1 = constrain(m1, THROTTLE_ARM, THROTTLE_MAX);
  m2 = constrain(m2, THROTTLE_ARM, THROTTLE_MAX);
  m3 = constrain(m3, THROTTLE_ARM, THROTTLE_MAX);
  m4 = constrain(m4, THROTTLE_ARM, THROTTLE_MAX);

  // Apply to motors only if armed and throttle is above minimum
  if (armed && baseThrottle > THROTTLE_MIN) {
    esc[0].writeMicroseconds(m1);
    esc[1].writeMicroseconds(m2);
    esc[2].writeMicroseconds(m3);
    esc[3].writeMicroseconds(m4);
  } else {
    // Disarmed or low throttle - motors OFF and reset integral
    for(int i = 0; i < 4; i++) {
      esc[i].writeMicroseconds(THROTTLE_ARM);
    }
    // Reset integrators when not flying
    IErrRoll = 0;
    IErrPitch = 0;
    IErrYaw = 0;
  }
}

// PID Position/Angle Controller - OUTER LOOP
// Takes desired angles and outputs desired angular rates for PID_Gyro
// This creates a cascaded PID control system for better stability
// Inputs: angle commands (degrees) for roll and pitch
// Outputs: rate commands (deg/s) that feed into PID_Gyro
void PID_Position(float angleRollCmd, float anglePitchCmd, float &rateRollOut, float &ratePitchOut, float dt) {
  // Calculate angle errors
  float errRoll_pos = angleRollCmd - angleRoll;
  float errPitch_pos = anglePitchCmd - anglePitch;

  // ROLL Position PID
  // Integral with anti-windup (optional for position loop)
  IErrRoll_pos += errRoll_pos * dt;
  IErrRoll_pos = constrain_float(IErrRoll_pos, -50.0, 50.0);
  // Derivative
  float DErrRoll_pos = (errRoll_pos - prevErrRoll_pos) / dt;
  prevErrRoll_pos = errRoll_pos;
  // PID output is desired rate
  rateRollOut = KpR_pos * errRoll_pos + KiR_pos * IErrRoll_pos + KdR_pos * DErrRoll_pos;
  rateRollOut = constrain_float(rateRollOut, -MAX_RATE_OUTPUT, MAX_RATE_OUTPUT);

  // PITCH Position PID
  IErrPitch_pos += errPitch_pos * dt;
  IErrPitch_pos = constrain_float(IErrPitch_pos, -50.0, 50.0);
  float DErrPitch_pos = (errPitch_pos - prevErrPitch_pos) / dt;
  prevErrPitch_pos = errPitch_pos;
  ratePitchOut = KpP_pos * errPitch_pos + KiP_pos * IErrPitch_pos + KdP_pos * DErrPitch_pos;
  ratePitchOut = constrain_float(ratePitchOut, -MAX_RATE_OUTPUT, MAX_RATE_OUTPUT);
}

// Read IMU and update attitude estimates - OPTIMIZED
// Updates global variables: gyroRoll, gyroPitch, gyroYaw, angleRoll, anglePitch
void updateIMU() {
  sensors_event_t accel, gyro, mag;
  icm.getEvent(&accel, &gyro, &mag);

  // Store gyro rates in deg/s (convert from rad/s)
  gyroRoll = gyro.gyro.x * 57.2957795;   // rad/s to deg/s
  gyroPitch = gyro.gyro.y * 57.2957795;
  gyroYaw = gyro.gyro.z * 57.2957795;

  // Update Madgwick filter with sensor data
  filter.updateIMU(gyro.gyro.x, gyro.gyro.y, gyro.gyro.z,
                   accel.acceleration.x, accel.acceleration.y, accel.acceleration.z);

  // Get attitude angles from filter
  angleRoll = filter.getRoll();
  anglePitch = filter.getPitch();
}
