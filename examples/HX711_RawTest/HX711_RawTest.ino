/*
 * HX711 Raw Test
 *
 * Minimal hardware verification — prints raw ADC values every 500ms.
 * No calibration, no scale, no commands. Just checks if the chip responds.
 *
 * Wiring:
 *   HX711 DOUT -> D4
 *   HX711 SCK  -> D5
 */

#include <HX711.h>

#define DOUT_PIN 4
#define SCK_PIN  5

HX711 scale;

void setup() {
  Serial.begin(115200);
  Serial.println("HX711 Raw Test");

  scale.begin(DOUT_PIN, SCK_PIN);

  // if (scale.is_ready()) {
  //   Serial.println("HX711 found OK");
  // } else {
  //   Serial.println("HX711 NOT found - check wiring!");
  // }
}

void loop() {
  if (scale.is_ready()) {
    long raw = scale.read();
    // Serial.print("RAW: ");
    Serial.println(raw);
  } else {
    Serial.println("NOT READY");
  }
  delay(50);
}
