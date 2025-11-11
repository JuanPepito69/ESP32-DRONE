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

// Constantes moteurs
const int THROTTLE_MIN = 1020;  // Démarrage
const int THROTTLE_ARM = 1000;  // ARM
const int THROTTLE_MAX = 2000;

float lastUpdate=0;
unsigned long lastRX = 0;
bool armed = false;

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
  
  Serial.println("✓ Madgwick initialisé");
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
        Serial.println("✗ DISARMED");
      }
      
      // Appliquer commandes
      mixAndApply(&cmd);
      
    } else {
      Serial.println("✗ Checksum error");
    }
  }
  
  // Failsafe si pas de RX depuis 500ms
  if (millis() - lastRX > 5000 && armed) {
    failsafe();
  }
}