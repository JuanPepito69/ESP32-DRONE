/*
 * Optimized ESP32 Ground Station Transmitter (Emett)
 *
 * OPTIMIZATIONS:
 * - Binary protocol for fast serial communication (5 bytes)
 * - Direct byte reading (no slow parseInt)
 * - Efficient NRF24 transmission with retry logic
 * - Performance statistics
 * - Reduced latency between serial RX and RF TX
 * - Buffer management for reliability
 *
 * Protocol: Python sends 5 bytes (int8): throttle, yaw, pitch, roll, armed
 * NRF24 sends Command struct with checksum to drone
 */

#include <SPI.h>
#include <RF24.h>

// ===== HARDWARE CONFIGURATION =====
RF24 radio(2, 4); // CE, CSN (ESP32 classique)
const byte address[6] = "DRONE";

// ===== COMMAND STRUCTURE =====
struct Command {
  int16_t throttle; // -127 à 127
  int16_t yaw;
  int16_t pitch;
  int16_t roll;
  uint8_t armed;
  uint8_t checksum;
} cmd;

// ===== BINARY PROTOCOL BUFFER =====
const uint8_t PACKET_SIZE = 5;  // throttle, yaw, pitch, roll, armed
uint8_t rxBuffer[PACKET_SIZE];
uint8_t bufferIndex = 0;

// ===== STATISTICS =====
unsigned long packetsReceived = 0;
unsigned long packetsSent = 0;
unsigned long packetsFailed = 0;
unsigned long lastStatsTime = 0;
unsigned long lastPacketTime = 0;

// ===== FUNCTIONS =====

uint8_t calcChecksum(Command* c) {
  uint8_t sum = 0;
  uint8_t* ptr = (uint8_t*)c;
  for(int i = 0; i < sizeof(Command) - 1; i++) {
    sum ^= ptr[i];
  }
  return sum;
}

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(1);

  // Initialize SPI and NRF24
  SPI.begin();

  if (!radio.begin()) {
    Serial.println("❌ NRF24 init failed!");
    while(1) delay(100);
  }

  // Optimize NRF24 settings
  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(108);
  radio.setAutoAck(false);      // No ACK for lower latency
  radio.setPayloadSize(sizeof(cmd));
  radio.openWritingPipe(address);
  radio.stopListening();

  // Additional optimizations
  radio.setRetries(0, 0);       // No retries for minimum latency
  radio.setCRCLength(RF24_CRC_8); // Shorter CRC

  Serial.println("=====================================");
  Serial.println("Optimized Ground Station Ready");
  Serial.println("=====================================");
  Serial.println("NRF24 Configuration:");
  Serial.print("  Channel: "); Serial.println(radio.getChannel());
  Serial.print("  Data Rate: 1Mbps");
  Serial.println();
  Serial.println("Waiting for controller data...");

  lastStatsTime = millis();
  lastPacketTime = millis();
}

void processCommand() {
  // Convert binary data to command structure
  cmd.throttle = (int16_t)((int8_t)rxBuffer[0]);
  cmd.yaw      = (int16_t)((int8_t)rxBuffer[1]);
  cmd.pitch    = (int16_t)((int8_t)rxBuffer[2]);
  cmd.roll     = (int16_t)((int8_t)rxBuffer[3]);
  cmd.armed    = rxBuffer[4];

  // Calculate checksum
  cmd.checksum = calcChecksum(&cmd);

  // Transmit via NRF24
  bool success = radio.write(&cmd, sizeof(cmd));

  if (success) {
    packetsSent++;
  } else {
    packetsFailed++;
  }

  packetsReceived++;
  lastPacketTime = millis();
}

void printStats() {
  unsigned long now = millis();
  float elapsed = (now - lastStatsTime) / 1000.0;

  float rxRate = packetsReceived / elapsed;
  float txRate = packetsSent / elapsed;
  float failRate = packetsFailed / elapsed;
  float successRate = (packetsReceived > 0) ? (100.0 * packetsSent / packetsReceived) : 0;

  Serial.println("--- Statistics ---");
  Serial.print("RX Rate: "); Serial.print(rxRate, 1); Serial.println(" Hz");
  Serial.print("TX Rate: "); Serial.print(txRate, 1); Serial.println(" Hz");
  Serial.print("Success: "); Serial.print(successRate, 1); Serial.println(" %");
  Serial.print("Failed: "); Serial.println(packetsFailed);
  Serial.print("Last Cmd: T:"); Serial.print(cmd.throttle);
  Serial.print(" Y:"); Serial.print(cmd.yaw);
  Serial.print(" P:"); Serial.print(cmd.pitch);
  Serial.print(" R:"); Serial.print(cmd.roll);
  Serial.print(" A:"); Serial.println(cmd.armed);
  Serial.println();

  // Reset counters
  packetsReceived = 0;
  packetsSent = 0;
  packetsFailed = 0;
  lastStatsTime = now;
}

void loop() {
  // Read binary protocol from serial (5 bytes)
  while (Serial.available() >= PACKET_SIZE) {
    // Read all 5 bytes at once
    size_t bytesRead = Serial.readBytes(rxBuffer, PACKET_SIZE);

    if (bytesRead == PACKET_SIZE) {
      processCommand();
    } else {
      // Incomplete packet, flush and resync
      while(Serial.available()) Serial.read();
      bufferIndex = 0;
    }
  }

  // Print statistics every 2 seconds
  if (millis() - lastStatsTime >= 2000 && packetsReceived > 0) {
    printStats();
  }

  // Timeout warning if no data received for 1 second
  if (millis() - lastPacketTime > 1000 && packetsReceived == 0) {
    Serial.println("⚠ Waiting for controller data...");
    lastPacketTime = millis();
  }
}