/*
 * DSHOT300 Test for ESP32-S3 Drone
 *
 * This is a proof-of-concept implementation of DSHOT300 protocol for ESC control
 * using the ESP32-S3's RMT (Remote Control Transceiver) peripheral.
 *
 * DSHOT300 Protocol:
 * - Digital protocol (more reliable than PWM)
 * - 300kbit/s data rate (3.33µs per bit)
 * - 16-bit frame: 11 bits throttle (0-2047) + 1 bit telemetry + 4 bits CRC
 * - Special commands: 0-47 (motor beeps, direction, settings)
 * - Throttle range: 48-2047 (0 = stop, 48 = min throttle, 2047 = max throttle)
 *
 * Hardware Setup:
 * - Motor 1: GPIO 1  (Front-Right, CW)
 * - Motor 2: GPIO 2  (Back-Left, CW)
 * - Motor 3: GPIO 42 (Front-Left, CCW)
 * - Motor 4: GPIO 41 (Back-Right, CCW)
 *
 * RMT Configuration:
 * - One RMT channel per motor (4 channels total)
 * - 80MHz APB clock with divider for precise timing
 * - Bit encoding: 1 = 75% duty, 0 = 37.5% duty
 *
 * Usage:
 * 1. Upload this sketch to ESP32-S3
 * 2. Open Serial Monitor at 115200 baud
 * 3. Follow on-screen menu to test motors
 * 4. Send throttle values (0-2047) or special commands
 *
 * Safety:
 * - REMOVE PROPELLERS during testing!
 * - Motors will beep on startup (arming sequence)
 * - Throttle starts at 0 (motors off)
 */

#include <driver/rmt.h>

// Motor pin configuration (same as main drone code)
const int MOTOR_PINS[4] = {1, 2, 42, 41};
const rmt_channel_t RMT_CHANNELS[4] = {RMT_CHANNEL_0, RMT_CHANNEL_1, RMT_CHANNEL_2, RMT_CHANNEL_3};

// DSHOT300 timing constants
// At 300kbit/s, each bit is 3.33µs
// Logic 1: 2.5µs high, 0.83µs low  (T1H, T1L)
// Logic 0: 1.25µs high, 2.08µs low (T0H, T0L)

// RMT tick configuration (80MHz / 4 = 20MHz = 0.05µs per tick)
#define RMT_CLK_DIV 4

// DSHOT300 timing in RMT ticks (0.05µs per tick)
#define DSHOT_T1H  50   // 2.5µs high for bit 1
#define DSHOT_T1L  17   // 0.85µs low for bit 1
#define DSHOT_T0H  25   // 1.25µs high for bit 0
#define DSHOT_T0L  42   // 2.1µs low for bit 0

// DSHOT frame constants
#define DSHOT_THROTTLE_MIN 48
#define DSHOT_THROTTLE_MAX 2047
#define DSHOT_FRAME_SIZE 16  // 16 bits per frame

// DSHOT special commands (0-47)
#define DSHOT_CMD_MOTOR_STOP           0
#define DSHOT_CMD_BEEP1               1
#define DSHOT_CMD_BEEP2               2
#define DSHOT_CMD_BEEP3               3
#define DSHOT_CMD_BEEP4               4
#define DSHOT_CMD_BEEP5               5
#define DSHOT_CMD_ESC_INFO            6
#define DSHOT_CMD_SPIN_DIRECTION_1    7
#define DSHOT_CMD_SPIN_DIRECTION_2    8
#define DSHOT_CMD_3D_MODE_OFF         9
#define DSHOT_CMD_3D_MODE_ON          10
#define DSHOT_CMD_SETTINGS_REQUEST    11
#define DSHOT_CMD_SAVE_SETTINGS       12
#define DSHOT_CMD_SPIN_DIRECTION_NORMAL   20
#define DSHOT_CMD_SPIN_DIRECTION_REVERSED 21

// Global variables
rmt_item32_t dshotPacket[4][16];  // Pre-encoded packets for each motor
uint16_t motorThrottle[4] = {0, 0, 0, 0};  // Current throttle values
bool telemetryRequest = false;

/**
 * Calculate CRC4 checksum for DSHOT frame
 * @param frame 12-bit frame (11 bits throttle + 1 bit telemetry)
 * @return 4-bit CRC checksum
 */
uint8_t calculateCRC(uint16_t frame) {
  uint8_t crc = 0;
  uint16_t data = frame;

  for (int i = 0; i < 12; i++) {
    crc ^= (data & 0x01);
    data >>= 1;

    if (crc & 0x01) {
      crc ^= 0x0D;  // CRC polynomial
    }
    crc >>= 1;
  }

  return crc & 0x0F;
}

/**
 * Encode a DSHOT value into RMT items
 * @param value Throttle value (0-2047)
 * @param telemetry Telemetry request bit
 * @param packet Output RMT packet (16 items)
 */
void encodeDShotPacket(uint16_t value, bool telemetry, rmt_item32_t* packet) {
  // Constrain value
  if (value > DSHOT_THROTTLE_MAX) {
    value = DSHOT_THROTTLE_MAX;
  }

  // Build 12-bit frame: 11 bits value + 1 bit telemetry
  uint16_t frame = (value << 1) | (telemetry ? 1 : 0);

  // Calculate CRC
  uint8_t crc = calculateCRC(frame);

  // Build complete 16-bit packet: 12 bits frame + 4 bits CRC
  uint16_t packet16 = (frame << 4) | crc;

  // Encode each bit as RMT item (MSB first)
  for (int i = 0; i < 16; i++) {
    bool bit = (packet16 >> (15 - i)) & 0x01;

    if (bit) {
      // Logic 1: long high, short low
      packet[i].level0 = 1;
      packet[i].duration0 = DSHOT_T1H;
      packet[i].level1 = 0;
      packet[i].duration1 = DSHOT_T1L;
    } else {
      // Logic 0: short high, long low
      packet[i].level0 = 1;
      packet[i].duration0 = DSHOT_T0H;
      packet[i].level1 = 0;
      packet[i].duration1 = DSHOT_T0L;
    }
  }
}

/**
 * Initialize RMT peripheral for DSHOT300
 */
void initDShot() {
  for (int i = 0; i < 4; i++) {
    rmt_config_t config = {};
    config.rmt_mode = RMT_MODE_TX;
    config.channel = RMT_CHANNELS[i];
    config.gpio_num = (gpio_num_t)MOTOR_PINS[i];
    config.mem_block_num = 1;
    config.tx_config.loop_en = false;
    config.tx_config.carrier_en = false;
    config.tx_config.idle_output_en = true;
    config.tx_config.idle_level = RMT_IDLE_LEVEL_LOW;
    config.clk_div = RMT_CLK_DIV;

    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));
  }

  Serial.println("✓ RMT channels initialized for DSHOT300");
}

/**
 * Send DSHOT packet to a specific motor
 * @param motor Motor index (0-3)
 * @param value Throttle value (0-2047)
 * @param telemetry Request telemetry
 */
void sendDShotValue(int motor, uint16_t value, bool telemetry = false) {
  if (motor < 0 || motor >= 4) return;

  encodeDShotPacket(value, telemetry, dshotPacket[motor]);
  rmt_write_items(RMT_CHANNELS[motor], dshotPacket[motor], 16, false);
}

/**
 * Send DSHOT command to a specific motor
 * @param motor Motor index (0-3)
 * @param command Special command (0-47)
 * @param repeat Number of times to repeat (commands need multiple sends)
 */
void sendDShotCommand(int motor, uint8_t command, int repeat = 10) {
  if (motor < 0 || motor >= 4) return;
  if (command > 47) return;

  for (int i = 0; i < repeat; i++) {
    sendDShotValue(motor, command, false);
    delayMicroseconds(1000);  // Wait between command packets
  }
}

/**
 * Send the same throttle value to all motors
 * @param value Throttle value (0-2047)
 */
void sendAllMotors(uint16_t value) {
  for (int i = 0; i < 4; i++) {
    sendDShotValue(i, value, telemetryRequest);
    motorThrottle[i] = value;
  }
}

/**
 * Send DSHOT command to all motors
 * @param command Special command (0-47)
 */
void sendCommandAllMotors(uint8_t command) {
  Serial.printf("Sending command %d to all motors...\n", command);
  for (int i = 0; i < 4; i++) {
    sendDShotCommand(i, command, 10);
  }
  Serial.println("✓ Command sent");
}

/**
 * Arming sequence for ESCs
 */
void armESCs() {
  Serial.println("\n=== ESC Arming Sequence ===");
  Serial.println("Sending zero throttle...");

  for (int i = 0; i < 100; i++) {
    sendAllMotors(0);
    delay(10);
  }

  Serial.println("✓ ESCs should be armed and ready");
  Serial.println("✓ You should hear confirmation beeps");
  Serial.println("\n⚠ WARNING: Motors can now spin! ⚠\n");
}

/**
 * Test sequence: beep motors
 */
void testBeeps() {
  Serial.println("\n=== Motor Beep Test ===");

  for (int motor = 0; motor < 4; motor++) {
    Serial.printf("Motor %d beeping...\n", motor + 1);
    sendDShotCommand(motor, DSHOT_CMD_BEEP1, 10);
    delay(300);
  }

  Serial.println("✓ Beep test complete");
}

/**
 * Test sequence: gradual throttle ramp
 */
void testThrottleRamp() {
  Serial.println("\n=== Throttle Ramp Test ===");
  Serial.println("⚠ Motors will spin! Make sure propellers are removed!");
  Serial.println("Starting in 3 seconds...");
  delay(3000);

  // Ramp up
  Serial.println("Ramping up to 25% throttle...");
  for (uint16_t throttle = DSHOT_THROTTLE_MIN; throttle <= 500; throttle += 10) {
    sendAllMotors(throttle);
    Serial.printf("Throttle: %d\n", throttle);
    delay(100);
  }

  delay(1000);

  // Ramp down
  Serial.println("Ramping down...");
  for (int16_t throttle = 500; throttle >= 0; throttle -= 10) {
    sendAllMotors(throttle > 0 ? throttle : 0);
    Serial.printf("Throttle: %d\n", throttle);
    delay(100);
  }

  sendAllMotors(0);
  Serial.println("✓ Ramp test complete - motors stopped");
}

/**
 * Display help menu
 */
void printMenu() {
  Serial.println("\n╔════════════════════════════════════════╗");
  Serial.println("║    DSHOT300 Test Menu                 ║");
  Serial.println("╚════════════════════════════════════════╝");
  Serial.println("Commands:");
  Serial.println("  0-2047    : Set throttle (0=stop, 48=min, 2047=max)");
  Serial.println("  a         : Arm ESCs");
  Serial.println("  s         : Stop all motors (throttle 0)");
  Serial.println("  b         : Beep test (all motors)");
  Serial.println("  r         : Throttle ramp test");
  Serial.println("  t         : Toggle telemetry request");
  Serial.println("  m1-m4     : Test individual motor (e.g., 'm1 500')");
  Serial.println("  info      : Show current status");
  Serial.println("  help      : Show this menu");
  Serial.println("\nSpecial DSHOT Commands:");
  Serial.println("  cmd 1-5   : Beep 1-5");
  Serial.println("  cmd 7-8   : Spin direction");
  Serial.println("  cmd 9-10  : 3D mode off/on");
  Serial.println("  cmd 12    : Save settings");
  Serial.println("  cmd 20-21 : Normal/Reversed direction");
  Serial.println("════════════════════════════════════════\n");
}

/**
 * Display current status
 */
void printStatus() {
  Serial.println("\n=== Current Status ===");
  for (int i = 0; i < 4; i++) {
    Serial.printf("Motor %d (GPIO %d): %d\n", i + 1, MOTOR_PINS[i], motorThrottle[i]);
  }
  Serial.printf("Telemetry request: %s\n", telemetryRequest ? "ON" : "OFF");
  Serial.println("===================\n");
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n\n");
  Serial.println("╔═══════════════════════════════════════════╗");
  Serial.println("║  DSHOT300 Test - ESP32-S3 Drone         ║");
  Serial.println("║  Proof of Concept for Digital ESC       ║");
  Serial.println("╚═══════════════════════════════════════════╝");
  Serial.println();
  Serial.println("⚠⚠⚠  REMOVE PROPELLERS BEFORE TESTING!  ⚠⚠⚠");
  Serial.println();

  // Initialize DSHOT
  initDShot();

  // Initial safe state - send zeros
  Serial.println("Setting all motors to zero throttle...");
  for (int i = 0; i < 50; i++) {
    sendAllMotors(0);
    delay(20);
  }

  Serial.println("✓ Initialization complete");
  Serial.println("\nType 'help' for available commands");
  Serial.println("Type 'a' to arm ESCs\n");
}

void loop() {
  // Continuously send throttle values to maintain signal
  // DSHOT requires regular updates (typically at loop rate)
  static unsigned long lastSend = 0;
  unsigned long now = millis();

  if (now - lastSend >= 4) {  // Send at ~250Hz
    for (int i = 0; i < 4; i++) {
      sendDShotValue(i, motorThrottle[i], telemetryRequest);
    }
    lastSend = now;
  }

  // Process serial commands
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    input.toLowerCase();

    if (input.length() == 0) return;

    // Parse commands
    if (input == "help") {
      printMenu();
    }
    else if (input == "info") {
      printStatus();
    }
    else if (input == "a") {
      armESCs();
    }
    else if (input == "s") {
      Serial.println("Stopping all motors...");
      sendAllMotors(0);
      Serial.println("✓ All motors stopped");
    }
    else if (input == "b") {
      testBeeps();
    }
    else if (input == "r") {
      testThrottleRamp();
    }
    else if (input == "t") {
      telemetryRequest = !telemetryRequest;
      Serial.printf("Telemetry request: %s\n", telemetryRequest ? "ON" : "OFF");
    }
    else if (input.startsWith("cmd ")) {
      int cmd = input.substring(4).toInt();
      if (cmd >= 0 && cmd <= 47) {
        sendCommandAllMotors(cmd);
      } else {
        Serial.println("Error: Command must be 0-47");
      }
    }
    else if (input.startsWith("m")) {
      // Individual motor control: m1 500
      int motorNum = input.charAt(1) - '1';
      int spaceIdx = input.indexOf(' ');

      if (motorNum >= 0 && motorNum < 4 && spaceIdx > 0) {
        uint16_t value = input.substring(spaceIdx + 1).toInt();
        if (value <= DSHOT_THROTTLE_MAX) {
          motorThrottle[motorNum] = value;
          Serial.printf("Motor %d set to %d\n", motorNum + 1, value);
        } else {
          Serial.println("Error: Value must be 0-2047");
        }
      } else {
        Serial.println("Error: Format is 'm1 500' (motor 1-4, value 0-2047)");
      }
    }
    else if (input.toInt() > 0 || input == "0") {
      // Throttle value
      uint16_t value = input.toInt();
      if (value <= DSHOT_THROTTLE_MAX) {
        sendAllMotors(value);
        Serial.printf("All motors set to %d\n", value);
      } else {
        Serial.println("Error: Throttle must be 0-2047");
      }
    }
    else {
      Serial.println("Unknown command. Type 'help' for available commands.");
    }
  }
}
