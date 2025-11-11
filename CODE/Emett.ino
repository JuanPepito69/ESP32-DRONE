#include <SPI.h>
#include <RF24.h>

RF24 radio(2, 4); // CE, CSN (ESP32 classique)
const byte address[6] = "DRONE";

struct Command {
  int16_t throttle; // -127 à 127
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  uint8_t armed;
  uint8_t checksum;
} cmd;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(5);
  
  SPI.begin();
  radio.begin();
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(108);
  radio.setAutoAck(false);
  radio.setPayloadSize(sizeof(cmd));
  radio.openWritingPipe(address);
  radio.stopListening();
  
  Serial.println("Ground Station prêt");
}

uint8_t calcChecksum(Command* c) {
  uint8_t sum = 0;
  uint8_t* ptr = (uint8_t*)c;
  for(int i = 0; i < sizeof(Command) - 1; i++) {
    sum ^= ptr[i];
  }
  return sum;
}

void loop() {
  if (Serial.available() > 10) {
    // Parse: throttle,yaw,pitch,roll,armed
    cmd.throttle = Serial.parseInt();
    cmd.yaw = Serial.parseInt();
    cmd.pitch = Serial.parseInt();
    cmd.roll = Serial.parseInt();
    cmd.armed = Serial.parseInt();
    Serial.read(); // Clear '\n'
    
    cmd.checksum = calcChecksum(&cmd);
    
    radio.write(&cmd, sizeof(cmd));
  }
}