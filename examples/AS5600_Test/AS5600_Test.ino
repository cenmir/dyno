/*
 * AS5600 Magnetic Sensor Test
 *
 * Reads angular position from AS5600 magnetic sensor and prints degrees.
 * Use this to verify sensor connection before running the main Dyno application.
 *
 * Hardware Setup:
 * - VCC -> 5V
 * - GND -> GND
 * - SDA -> Pin 20 (Arduino Mega) or A4 (Uno)
 * - SCL -> Pin 21 (Arduino Mega) or A5 (Uno)
 * - Mount diametrically magnetized magnet 1-3mm from sensor
 *
 * See docs/AS5600_code_explanation.md for detailed explanation.
 */

#include "AS5600.h"
#include <Wire.h>

AS5600 as5600;

void setup() {
  Serial.begin(115200);
  Wire.begin();
  as5600.begin(3);  // Set direction pin

  Serial.print("AS5600 connected: ");
  Serial.println(as5600.isConnected() ? "Yes" : "No");

  delay(3000);
}

void loop() {
  int rawAngle = as5600.readAngle();
  float angle = map(rawAngle, 0, 4095, 0, 36000) / 100.0;
  Serial.println(angle);
  delay(100);
}
