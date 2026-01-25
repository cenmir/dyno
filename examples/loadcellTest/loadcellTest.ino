/*
 * Load Cell (HX711) Test and Calibration
 *
 * Standalone tool for testing and calibrating the load cell.
 * Use this before integrating with the main Dyno application.
 *
 * Commands:
 * - C or Calibrate: Run calibration procedure
 * - L: Start/stop continuous measurement
 * - T or Tare: Zero the scale
 * - R or Raw: Toggle raw/scaled output
 * - SetOffset <value>: Set offset manually
 * - SetScale <value>: Set scale manually
 * - GetOffset: Display current offset
 * - GetScale: Display current scale
 *
 * Hardware:
 * - HX711 DOUT -> Pin 2
 * - HX711 SCK -> Pin 3
 */

#include <HX711.h>

#define DATA_PIN  2
#define CLOCK_PIN 3

bool isMeasuring = false;
bool isRawVal = false;

HX711 LoadCell;

void setup() {
  Serial.begin(9600);

  Serial.println("Load Cell Test Firmware v1.3");
  Serial.print("HX711 Library: ");
  Serial.println(HX711_LIB_VERSION);
  Serial.println();

  LoadCell.begin(DATA_PIN, CLOCK_PIN);

  // Default calibration values - update after calibration!
  LoadCell.set_offset(-28047);
  LoadCell.set_scale(89.553596496582031f);

  Serial.println();
  DisplayHelp();
}

void loop() {
  ReadSerial();

  if (isMeasuring) {
    if (isRawVal) {
      Serial.println(LoadCell.read());
    } else {
      Serial.println(LoadCell.get_units(1));
    }
    delay(50);
  }
}

void ReadSerial() {
  if (Serial.available() == 0) { return; }

  const byte numChars = 32;
  char receivedChars[numChars];
  char tempChars[numChars];
  static byte ndx = 0;
  bool newData = false;

  while (Serial.available() > 0 && newData == false) {
    char receivedChar = Serial.read();
    if (receivedChar != '\n') {
      receivedChars[ndx] = receivedChar;
      ndx++;
      if (ndx >= numChars) {
        ndx = numChars - 1;
      }
    } else {
      receivedChars[ndx] = '\0';
      ndx = 0;
      newData = true;
    }
  }

  if (!newData) { return; }

  Serial.print("Received: ");
  Serial.println(receivedChars);

  strcpy(tempChars, receivedChars);
  char* strtokIndx;
  strtokIndx = strtok(tempChars, " ");
  char command[32] = { 0 };
  strcpy(command, strtokIndx);

  if (strcmp(command, "Calibrate") == 0 || strcmp(command, "C") == 0) {
    calibrate();
  }
  else if (strcmp(command, "L") == 0) {
    isMeasuring = !isMeasuring;
    Serial.println(isMeasuring ? "Measuring started" : "Measuring stopped");
  }
  else if (strcmp(command, "Start") == 0) {
    isMeasuring = true;
  }
  else if (strcmp(command, "Stop") == 0) {
    isMeasuring = false;
  }
  else if (strcmp(command, "Tare") == 0 || strcmp(command, "T") == 0) {
    LoadCell.tare();
    Serial.println("Tared!");
  }
  else if (strcmp(command, "SetOffset") == 0) {
    strtokIndx = strtok(NULL, " ");
    long val = atoi(strtokIndx);
    LoadCell.set_offset(val);
    Serial.print("Offset: ");
    Serial.println(LoadCell.get_offset());
  }
  else if (strcmp(command, "SetScale") == 0) {
    strtokIndx = strtok(NULL, " ");
    float float_val = atof(strtokIndx);
    LoadCell.set_scale(float_val);
    Serial.print("Scale: ");
    Serial.println(LoadCell.get_scale());
  }
  else if (strcmp(command, "GetOffset") == 0) {
    Serial.print("Offset: ");
    Serial.println(LoadCell.get_offset());
  }
  else if (strcmp(command, "GetScale") == 0) {
    Serial.print("Scale: ");
    Serial.println(LoadCell.get_scale());
  }
  else if (strcmp(command, "Raw") == 0 || strcmp(command, "R") == 0) {
    isRawVal = !isRawVal;
    Serial.println(isRawVal ? "Displaying raw values" : "Displaying scaled values");
  }
  else {
    Serial.println("Unrecognized command.");
    DisplayHelp();
  }
}

void DisplayHelp() {
  Serial.println("----------------------");
  Serial.println("Valid Commands:");
  Serial.println("  C - Calibrate");
  Serial.println("  L - Start/stop measuring");
  Serial.println("  T - Tare (zero)");
  Serial.println("  R - Toggle raw/scaled");
  Serial.println("  SetOffset <int>");
  Serial.println("  SetScale <float>");
  Serial.println("  GetOffset");
  Serial.println("  GetScale");
  Serial.println("----------------------");
}

void calibrate() {
  Serial.println("\n\nCALIBRATION\n===========");
  Serial.println("1. Remove all weight from the load cell.");

  while (Serial.available()) Serial.read();

  Serial.println("2. Press Enter to begin.");
  while (Serial.available() == 0);
  while (Serial.read() != '\n');

  Serial.println("Determining zero weight offset...");
  LoadCell.tare(20);
  long offset = LoadCell.get_offset();

  Serial.print("OFFSET: ");
  Serial.println(offset);
  Serial.println();

  Serial.println("3. Place a known weight on the load cell.");
  while (Serial.available()) Serial.read();

  Serial.println("4. Enter the weight in kilograms (e.g., 0.5) and press Enter.");
  while (Serial.available() == 0);
  float known_weight = Serial.parseFloat();

  Serial.print("KNOWN WEIGHT (kg): ");
  Serial.println(known_weight, 4);

  LoadCell.calibrate_scale(known_weight, 20);
  float scale = LoadCell.get_scale();

  Serial.print("SCALE: ");
  Serial.println(scale, 15);

  Serial.println("\nCalibration complete! Add these lines to your setup():");
  Serial.print("LoadCell.set_offset(");
  Serial.print(offset);
  Serial.print("L);\nLoadCell.set_scale(");
  Serial.print(scale, 15);
  Serial.print("f);\n\n");
}
