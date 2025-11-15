#include <SPI.h>
#include <RF24.h>
#include <ESP32Servo.h>
#include <Adafruit_ICM20948.h>
#include <Adafruit_Sensor.h>
#include <MadgwickAHRS.h>
#include <Wire.h>

float gyroOffsetX = 0, gyroOffsetY = 0, gyroOffsetZ = 0;
float accelOffsetX = 0, accelOffsetY = 0, accelOffsetZ = 0;


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
float dt=0.001;
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
const float KdY = 2;
float IErrYaw = 0;
float prevErrYaw = 0;

// Integral limits (anti-windup)
const float I_LIMIT = 400.0;

// POSITION PID (Angle) - Outer loop
// ROLL Angle
const float KpR_pos = 2.0;
const float KiR_pos = 0.0;
const float KdR_pos = 2;
float IErrRoll_pos = 0;
float prevErrRoll_pos = 0;

// PITCH Angle
const float KpP_pos = 2.0;
const float KiP_pos = 0.0;
const float KdP_pos = 2;
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

float lastUpdate=0;
unsigned long lastRX = 0;
bool armed = false;

void calibrateIMU() {
  Serial.println("=== CALIBRATION IMU ===");
  Serial.println("Poser drone sur surface PARFAITEMENT PLATE");
  Serial.println("Ne pas toucher pendant 10 sec...");
  delay(3000);
  
  // 1. CALIBRATION GYRO
  float gSumX = 0, gSumY = 0, gSumZ = 0;
  float aSumX = 0, aSumY = 0, aSumZ = 0;
  const int samples = 1000;
  
  for(int i = 0; i < samples; i++) {
    sensors_event_t a, g, m;
    icm.getEvent(&a, &g, &m);
    
    // Accumule gyro
    gSumX += g.gyro.x;
    gSumY += g.gyro.y;
    gSumZ += g.gyro.z;
    
    // Accumule accel
    aSumX += a.acceleration.x;
    aSumY += a.acceleration.y;
    aSumZ += a.acceleration.z;
    
    delay(2);
  }
  
  // Moyennes gyro
  gyroOffsetX = gSumX / samples;
  gyroOffsetY = gSumY / samples;
  gyroOffsetZ = gSumZ / samples;
  
  // Moyennes accel (avec compensation gravité sur Z)
  accelOffsetX = aSumX / samples;
  accelOffsetY = aSumY / samples;
  accelOffsetZ = (aSumZ / samples) - 9.81;  // Retirer 1G
  
  Serial.println("✓ Calibration terminée!");
  Serial.print("Gyro offsets: ");
  Serial.print(gyroOffsetX, 4); Serial.print(", ");
  Serial.print(gyroOffsetY, 4); Serial.print(", ");
  Serial.println(gyroOffsetZ, 4);
  
  Serial.print("Accel offsets: ");
  Serial.print(accelOffsetX, 3); Serial.print(", ");
  Serial.print(accelOffsetY, 3); Serial.print(", ");
  Serial.println(accelOffsetZ, 3);
  
  // 2. VERIFICATION ANGLE INITIAL
  delay(100);
  updateIMU();
  Serial.print("Angle initial Roll: ");
  Serial.print(angleRoll);
  Serial.print("° Pitch: ");
  Serial.print(anglePitch);
  Serial.println("°");
  
  if(abs(angleRoll) > 2 || abs(anglePitch) > 2) {
    Serial.println("⚠️ ATTENTION: Surface pas plate ou IMU mal monté!");
  }
}


void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  while (!icm.begin_I2C(0x68, &Wire)) {
    Serial.println("ICM20948 non trouvé!");
    delay(50);
  }
  // 1. RANGES
  icm.setAccelRange(ICM20948_ACCEL_RANGE_8_G);
  icm.setGyroRange(ICM20948_GYRO_RANGE_1000_DPS);
  
  // 2. DATA RATES
  icm.setAccelRateDivisor(0);
  icm.setGyroRateDivisor(0);
  
  // 3. FILTRES DLPF avec les bonnes constantes
  icm.enableAccelDLPF(true, ICM20X_ACCEL_FREQ_50_4_HZ);   // 50Hz
  icm.enableGyrolDLPF(true, ICM20X_GYRO_FREQ_51_2_HZ);    // 51Hz
  
  // 4. Calibration
  calibrateIMU();
  
  filter.begin(500);
  
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

void loop() {
  float ti = micros();
  updateIMU();
  debug();
  dt=2.0/1000.0;
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
      // PID CASCADE

      float raterollcmd=0;
      float ratepitchcmd=0;
      float angleRollCmd = mapf(cmd.roll, -127, 127, -30, 30);  // ±30°
      float anglePitchCmd = mapf(cmd.pitch, -127, 127, -30, 30);
      float rateYawCmd = mapf(cmd.yaw, -127, 127, -180, 180);   // deg/s
      int baseThrottle = map(cmd.throttle, -127, 127, THROTTLE_ARM, THROTTLE_MAX);

      PID_Position(angleRollCmd,anglePitchCmd,raterollcmd,ratepitchcmd,dt);
      PID_Gyro(raterollcmd,ratepitchcmd,rateYawCmd,baseThrottle,dt);
      
    } else {
      Serial.println("Checksum error");
      
    }
  }
  
  // Failsafe si pas de RX depuis 500ms
  if (millis() - lastRX > 5000 && armed) {
    failsafe();
  }
  if((micros()-ti)/1000000.0<dt){
    delayMicroseconds(dt*1000000.0-(micros()-ti));
  }
  Serial.println(1/((micros()-ti)/1000000.0));
  
}

// PID POSITION CASCADE ETAPE 1
void PID_Position(float angleRollCmd, float anglePitchCmd, float &rateRollOut, float &ratePitchOut, float dt) {
  // Erreurs : 

  float errRoll_pos=angleRollCmd-angleRoll;
  float errPitch_pos=anglePitchCmd-anglePitch;

  // Termes Dérivée et Integrale
  IErrRoll_pos+=errRoll_pos*dt;
  IErrRoll_pos=constrain_float(IErrRoll_pos,-I_LIMIT,I_LIMIT);
  float DErrRoll_pos=(errRoll_pos-prevErrRoll_pos)/dt;
  prevErrRoll_pos=errRoll_pos; 

  IErrPitch_pos+=errPitch_pos*dt;
  IErrPitch_pos=constrain_float(IErrPitch_pos,-I_LIMIT,I_LIMIT);
  float DErrPitch_pos=(errPitch_pos-prevErrPitch_pos)/dt;
  prevErrPitch_pos=errPitch_pos;

  rateRollOut=KpR_pos*errRoll_pos+IErrRoll_pos*KiR_pos+KdR_pos*DErrRoll_pos;
  rateRollOut=constrain_float(rateRollOut,-MAX_RATE_OUTPUT,+MAX_RATE_OUTPUT);

  ratePitchOut=KpP_pos*errPitch_pos+IErrPitch_pos*KiP_pos+KdP_pos*DErrPitch_pos;
  ratePitchOut=constrain_float(ratePitchOut,-MAX_RATE_OUTPUT,+MAX_RATE_OUTPUT);
}


void PID_Gyro(float rateRollCmd, float ratePitchCmd, float rateYawCmd, int baseThrottle, float dt) {
  // Calcul des erreurs
  float ErrYaw = rateYawCmd-gyroYaw;
  float ErrPitch = ratePitchCmd-gyroPitch;
  float ErrRoll = rateRollCmd-gyroRoll;
  
  // Yaw
  IErrYaw += ErrYaw * dt;// Ajouter l'erreur au terme / Vecteur integral
  IErrYaw=constrain_float(IErrYaw,-I_LIMIT,I_LIMIT);
  float DErrYaw = (ErrYaw - prevErrYaw)/ dt;// Ajouter l'erreur au terme / Vecteur Derivee
  int yawEffect = KpY * ErrYaw + KiY * IErrYaw + KdY * DErrYaw;
  prevErrYaw=ErrYaw;

  // Roll
  IErrRoll += ErrRoll * dt;// Ajouter l'erreur au terme / Vecteur integral
  IErrRoll=constrain_float(IErrRoll,-I_LIMIT,I_LIMIT);
  float DErrRoll = (ErrRoll - prevErrRoll)  / dt;// Ajouter l'erreur au terme / Vecteur Derivee
  int rollEffect = KpR * ErrRoll + KiR * IErrRoll + KdR * DErrRoll;
  prevErrRoll=ErrRoll; 

  // Pitch
  IErrPitch = IErrPitch + ErrPitch * dt; // Ajouter l'erreur au terme / Vecteur integral
  IErrPitch = constrain_float(IErrPitch,-I_LIMIT,I_LIMIT);
  float DErrPitch = (ErrPitch-prevErrPitch) / dt;  // Ajouter l'erreur au terme / Vecteur Derivee
  int pitchEffect = KpP * ErrPitch + KiP * IErrPitch + KdP * DErrPitch;
  prevErrPitch=ErrPitch;

  // Fusion avec Throttle
  //  1(CW) 3(CCW)
  //     \ /
  //     / \                                                                                                                                                                            \
  //  4(CCW) 2(CW)

  // Motor 1 (avant-gauche, CW): -yaw for CW motors
  int m1 = baseThrottle + rollEffect + pitchEffect - yawEffect;
  m1 = constrain(m1,THROTTLE_MIN,THROTTLE_MAX);
  // Motor 2 (arrière-droite, CW): -yaw for CW motors
  int m2 = baseThrottle - rollEffect - pitchEffect - yawEffect;
  m2 = constrain(m2,THROTTLE_MIN,THROTTLE_MAX);
  // Motor 3 (avant-droite, CCW): +yaw for CCW motors
  int m3 = baseThrottle - rollEffect + pitchEffect + yawEffect;
  m3=constrain(m3,THROTTLE_MIN,THROTTLE_MAX);
  // Motor 4 (arrière-gauche, CCW): +yaw for CCW motors
  int m4 = baseThrottle + rollEffect - pitchEffect + yawEffect;
  m4=constrain(m4,THROTTLE_MIN,THROTTLE_MAX);

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

void PID_Descente(float dt,float &cmdThrottle){
// PID POUR GERER LES GAZS SI LE DRONE EST EN FAILSAFE :
// IL FAUT MESURE PRESSION ATMO AU DEMARRAGE DU DRONE POUR ESTIMER H0
// ET SOIT COMMANDER THROTTLE EN FONCTION DE H0-H
// SOIT CONTROLER LA VITESSE DE DESCENTE EN FAISANT UNE MESURE DE VITESSE DE DESCENTE AVEC H(T)-H(T-1) ET ASSERVIR CETTE VITESSE POUR AVOIR UN ATERISSAGE CONTROLE
// CETTE CMD THROTTLE SERA ENSUITE ENVOYE A PID GYRO AVEC LES INFO CMDRATEGYRO=0 ET CE THROTTLE.

}

void updateIMU() {
  sensors_event_t accel, gyro, mag;
  icm.getEvent(&accel, &gyro, &mag);
  
  // Applique TOUS les offsets
  float gx = gyro.gyro.x - gyroOffsetX;
  float gy = gyro.gyro.y - gyroOffsetY;
  float gz = gyro.gyro.z - gyroOffsetZ;
  
  float ax = accel.acceleration.x - accelOffsetX;
  float ay = accel.acceleration.y - accelOffsetY;
  float az = accel.acceleration.z - accelOffsetZ;
  
  // Convertir gyro en deg/s
  gyroRoll = gx * 57.2958;
  gyroPitch = gy * 57.2958;
  gyroYaw = gz * 57.2958;
  
  // Update Madgwick avec données CORRIGÉES
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  
  angleRoll = filter.getPitch();
  anglePitch = filter.getRoll();
}

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

void debug() {
  Serial.print("Gyro: ");
  Serial.print(gyroRoll); Serial.print(" ");
  Serial.print(gyroPitch); Serial.print(" ");
  Serial.print(gyroYaw); Serial.print(" | Angle: ");
  Serial.print(angleRoll); Serial.print(" ");
  Serial.println(anglePitch);
}