/*
 * Dyno - Stepper Motor Dynamometer
 *
 * Open-loop stepper speed control with AS5600 RPM measurement
 * and HX711 load cell for torque measurement.
 *
 * Features:
 * - Open-loop speed control (steppers hold speed until stall)
 * - AS5600 magnetic sensor for RPM measurement & stall detection
 * - HX711-based load cell for torque measurement
 * - SerialCommander for easy control
 * - Automated test with progressive brake loading
 *
 * Hardware:
 * - RAMPS 1.4 shield
 * - Test motor on X-axis
 * - Brake actuator motor on Y-axis
 * - AS5600 on I2C (SDA=20, SCL=21 on RAMPS)
 * - HX711 on Digital Pins (D4, D5)
 */

#define VERSION "2.0"

#include <Arduino.h>
#include <AccelStepper.h>
#include <Wire.h>
#include "AS5600.h"
#include <HX711.h>
#include "SerialCommander.h"

// === PIN DEFINITIONS ===
#define X_STEP_PIN         54
#define X_DIR_PIN          55
#define X_ENABLE_PIN       38

#define Y_STEP_PIN         60
#define Y_DIR_PIN          61
#define Y_ENABLE_PIN       56

#define LOADCELL_DOUT_PIN  4
#define LOADCELL_SCK_PIN   5

// === MOTOR & PHYSICAL CONSTANTS ===
#define MAX_SPEED          10000
#define ACCELERATION       2000

#define TEST_MOTOR_STEPS_PER_REV    (200 * 8)  // 200 full steps × 8 microsteps (TMC2226, no jumpers)
#define BRAKE_MOTOR_STEPS_PER_REV   (200 * 8)  // 200 full steps × 8 microsteps (TMC2226, no jumpers)

#define ARM_LENGTH_METERS  0.1f   // IMPORTANT: Measure and update this value!
#define GRAVITY            9.80665f

// === GLOBAL OBJECTS ===
AccelStepper testMotor = AccelStepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper brakeMotor = AccelStepper(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AS5600 as5600;
HX711 LoadCell;
SerialCommander cmd;

// === GLOBAL STATE ===
float targetRPM = 0;
float currentRPM = 0;
unsigned long lastRpmTime = 0;
unsigned long lastStatusTime = 0;
unsigned long lastSensorTime = 0;
bool autoStatus = false;
bool autoSensor = false;
bool autoLoadCell = false;
unsigned long lastLoadCellTime = 0;
long brakePosition = 0;

// === AUTOMATED TEST STATE ===
bool testRunning = false;
float testTargetRPM = 0;
float testMaxTorque = 0;
float testTorqueAtStall = 0;
unsigned long testStallStartTime = 0;
unsigned long testLastBrakeTime = 0;
int testBrakeIncrement = 50;       // Steps per brake increment
unsigned long testBrakeInterval = 500; // ms between brake increments
float testStallThreshold = 0.90;   // Stall if RPM < 90% of target
unsigned long testStallDuration = 1000; // ms below threshold to confirm stall

// === FORWARD DECLARATIONS ===
void updateRPM();
float getRPM();
void setMotorRPM(float rpm);
void stopMotor();
void displayStatus();
float getTorque();
void calibrate();
void handleSetRPM(SerialCommander* c);
void handleBrake(SerialCommander* c);
void handleBrakeApply(SerialCommander* c);
void handleBrakeRelease(SerialCommander* c);
void handleBrakeHome(SerialCommander* c);
void handleStatus(SerialCommander* c);
void handleStop(SerialCommander* c);
void handleAutoStatus(SerialCommander* c);
void handleReadTorque(SerialCommander* c);
void handleReadLoad(SerialCommander* c);
void handleDebugLoadCell(SerialCommander* c);
void handleTare(SerialCommander* c);
void handleCalibrate(SerialCommander* c);
void handleHelp(SerialCommander* c);
void handleRunTest(SerialCommander* c);
void handleAbortTest(SerialCommander* c);
void handleDebug(SerialCommander* c);
void handleEnable(SerialCommander* c);
void handleDisable(SerialCommander* c);
void handleReadSensor(SerialCommander* c);
void handleSetSpeed(SerialCommander* c);
void runTestStep();


void setup() {
  Serial.begin(115200);
  Wire.begin();

  // --- Initialize AS5600 ---
  as5600.begin(3);
  Serial.print("AS5600 connected: ");
  Serial.println(as5600.isConnected() ? "Yes" : "No");

  // --- Initialize Load Cell (HX711) ---
  LoadCell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  // IMPORTANT: Run 'calibrate' and update these values!
  LoadCell.set_offset(0);
  LoadCell.set_scale(1.0f);
  Serial.print("HX711 initialized. Ready for calibration ('tare' then 'calibrate').\n");

  // --- Configure Motors ---
  testMotor.setEnablePin(X_ENABLE_PIN);
  testMotor.setPinsInverted(true, false, true);  // DIR inverted
  testMotor.disableOutputs();
  testMotor.setMaxSpeed(MAX_SPEED);

  brakeMotor.setEnablePin(Y_ENABLE_PIN);
  brakeMotor.setPinsInverted(false, false, true);
  brakeMotor.disableOutputs();
  brakeMotor.setMaxSpeed(MAX_SPEED);
  brakeMotor.setAcceleration(ACCELERATION);

  // --- Register Commands ---
  cmd.begin("Dyno Ready - Open-Loop Control v" VERSION);
  cmd.addCommand("setRPM", handleSetRPM);
  cmd.addCommand("brake", handleBrake);
  cmd.addCommand("brakeApply", handleBrakeApply);
  cmd.addCommand("brakeRelease", handleBrakeRelease);
  cmd.addCommand("brakeHome", handleBrakeHome);
  cmd.addCommand("status", handleStatus);
  cmd.addCommand("autoStatus", handleAutoStatus);
  cmd.addCommand("readTorque", handleReadTorque);
  cmd.addCommand("readLoad", handleReadLoad);
  cmd.addCommand("debugLoad", handleDebugLoadCell);
  cmd.addCommand("tare", handleTare);
  cmd.addCommand("calibrate", handleCalibrate);
  cmd.addCommand("stop", handleStop);
  cmd.addCommand("help", handleHelp);
  cmd.addCommand("runTest", handleRunTest);
  cmd.addCommand("abortTest", handleAbortTest);
  cmd.addCommand("debug", handleDebug);
  cmd.addCommand("enable", handleEnable);
  cmd.addCommand("disable", handleDisable);
  cmd.addCommand("readSensor", handleReadSensor);
  cmd.addCommand("setSpeed", handleSetSpeed);

  Serial.println("Type 'help' for available commands");
}

void loop() {
  cmd.process();
  updateRPM();
  testMotor.runSpeed();
  brakeMotor.run();

  // Run automated test step if active
  if (testRunning) {
    runTestStep();
  }

  if (autoSensor && (millis() - lastSensorTime > 300)) {
    int angle = as5600.readAngle();
    float degrees = angle * 360.0 / 4096.0;
    Serial.print("Angle: ");
    Serial.print(angle);
    Serial.print(" (");
    Serial.print(degrees, 1);
    Serial.print(" deg) | RPM: ");
    Serial.println(getRPM(), 2);
    lastSensorTime = millis();
  }

  if (autoLoadCell && (millis() - lastLoadCellTime > 300)) {
    if (LoadCell.is_ready()) {
      Serial.print("RAW:");
      Serial.print(LoadCell.read());
      Serial.print(" | UNITS:");
      Serial.println(LoadCell.get_units(1), 4);
    }
    lastLoadCellTime = millis();
  }

  if (autoStatus && (millis() - lastStatusTime > 500)) {
    displayStatus();
    lastStatusTime = millis();
  }
}

// ==================== RPM & Motor Helpers ====================

void updateRPM() {
  if (millis() - lastRpmTime >= 100) {
    currentRPM = -as5600.getAngularSpeed(AS5600_MODE_DEGREES) / 6.0;
    lastRpmTime = millis();
  }
}

float getRPM() {
  return currentRPM;
}

void setMotorRPM(float rpm) {
  targetRPM = rpm;
  float stepsPerSec = rpm * TEST_MOTOR_STEPS_PER_REV / 60.0;
  testMotor.setSpeed(stepsPerSec);
}

void stopMotor() {
  targetRPM = 0;
  testMotor.setSpeed(0);
}

// ==================== Command Handlers ====================

void handleSetRPM(SerialCommander* c) {
  float rpm = c->getFloat(0);
  setMotorRPM(rpm);
  Serial.print("RPM set to: ");
  Serial.print(rpm, 0);
  Serial.print(" (");
  Serial.print(rpm * TEST_MOTOR_STEPS_PER_REV / 60.0, 0);
  Serial.println(" steps/s)");
}

void handleBrake(SerialCommander* c) {
  long steps = c->getInt(0);
  brakeMotor.move(steps);
  brakePosition += steps;
  Serial.print("Brake motor moving ");
  Serial.print(steps);
  Serial.print(" steps. New position: ");
  Serial.println(brakePosition);
}

void handleBrakeApply(SerialCommander* c) {
  long steps = (c->getParamCount() > 0) ? c->getInt(0) : 500;
  brakeMotor.move(steps);
  brakePosition += steps;
  Serial.print("Applying brake: +");
  Serial.print(steps);
  Serial.print(" steps | New position: ");
  Serial.println(brakePosition);
}

void handleBrakeRelease(SerialCommander* c) {
    long steps = (c->getParamCount() > 0) ? c->getInt(0) : 500;
    brakeMotor.move(-steps);
    brakePosition -= steps;
    Serial.print("Releasing brake: -");
    Serial.print(steps);
    Serial.print(" steps | New position: ");
    Serial.println(brakePosition);
}

void handleBrakeHome(SerialCommander* c) {
  Serial.print("Returning brake to home (0) from ");
  Serial.println(brakePosition);
  brakeMotor.moveTo(0);
  brakePosition = 0;
}

void handleStatus(SerialCommander* c) {
  displayStatus();
}

void handleAutoStatus(SerialCommander* c) {
  autoStatus = (c->getParamCount() > 0) ? c->getBool(0) : !autoStatus;
  Serial.print("Auto status display: ");
  Serial.println(autoStatus ? "ON" : "OFF");
}

void handleReadTorque(SerialCommander* c) {
  Serial.print("Torque: ");
  Serial.print(getTorque(), 4);
  Serial.println(" Nm");
}

void handleReadLoad(SerialCommander* c) {
  if (LoadCell.is_ready()) {
    float m = LoadCell.get_units(1); //kg
    Serial.print("Load: ");
    Serial.print(m, 4);
    Serial.print(" kg | Force: ");
    Serial.print(m * GRAVITY, 4);
    Serial.print(" N | Torque: ");
    Serial.print(m * GRAVITY * ARM_LENGTH_METERS, 4);
    Serial.println(" Nm");
  } else {
    Serial.println("Load cell not ready.");
  }
}

void handleDebugLoadCell(SerialCommander* c) {
  autoLoadCell = !autoLoadCell;
  Serial.print("Load cell debug: ");
  Serial.println(autoLoadCell ? "ON" : "OFF");
}

void handleTare(SerialCommander* c) {
  Serial.println("Taring load cell... do not touch.");
  LoadCell.tare(10);
  Serial.println("Tare complete.");
}

void handleCalibrate(SerialCommander* c) {
  calibrate();
}

void handleDebug(SerialCommander* c) {
  Serial.print("AS5600 angle: ");
  Serial.print(as5600.readAngle());
  Serial.print(" | raw: ");
  Serial.print(as5600.rawAngle());
  Serial.print(" | speed: ");
  Serial.print(testMotor.speed(), 1);
  Serial.print(" steps/s | pos: ");
  Serial.print(testMotor.currentPosition());
  Serial.print(" | RPM: ");
  Serial.println(getRPM(), 2);
}

void handleReadSensor(SerialCommander* c) {
  if (c->getParamCount() > 0 && c->getString(0)[0] == 'C') {
    autoSensor = !autoSensor;
    Serial.print("Continuous sensor: ");
    Serial.println(autoSensor ? "ON" : "OFF");
    return;
  }
  int angle = as5600.readAngle();
  float degrees = angle * 360.0 / 4096.0;
  Serial.print("Angle: ");
  Serial.print(angle);
  Serial.print(" (");
  Serial.print(degrees, 1);
  Serial.print(" deg) | RPM: ");
  Serial.println(getRPM(), 2);
}

void handleSetSpeed(SerialCommander* c) {
  float stepsPerSec = c->getFloat(0);
  targetRPM = stepsPerSec * 60.0 / TEST_MOTOR_STEPS_PER_REV;
  testMotor.setSpeed(stepsPerSec);
  Serial.print("Open-loop speed: ");
  Serial.print(stepsPerSec, 0);
  Serial.println(" steps/s");
}

void handleEnable(SerialCommander* c) {
  testMotor.enableOutputs();
  brakeMotor.enableOutputs();
  Serial.println("All motors enabled.");
}

void handleDisable(SerialCommander* c) {
  stopMotor();
  testMotor.disableOutputs();
  brakeMotor.disableOutputs();
  Serial.println("All motors disabled.");
}

void handleStop(SerialCommander* c) {
  stopMotor();
  brakeMotor.stop();
  Serial.println("EMERGENCY STOP - All motors stopped");
}

void handleHelp(SerialCommander* c) {
  Serial.println("\n=== Dyno v" VERSION " - Commands ===");
  Serial.println("\n--- Automated Testing ---");
  Serial.println("runTest <rpm>           - Run automated torque test at specified RPM");
  Serial.println("abortTest               - Abort running test");

  Serial.println("\n--- Test Motor ---");
  Serial.println("setRPM <value>          - Set motor speed in RPM (0 to stop)");
  Serial.println("setSpeed <steps/s>      - Set motor speed in steps/s (raw)");
  Serial.println("status                  - Display current system status");
  Serial.println("autoStatus [true/false] - Toggle auto status display");

  Serial.println("\n--- Brake Control ---");
  Serial.println("brake <steps>           - Move brake motor (+apply / -release)");
  Serial.println("brakeApply [steps]      - Apply brake (default: 500)");
  Serial.println("brakeRelease [steps]    - Release brake (default: 500)");
  Serial.println("brakeHome               - Return brake to home position (0)");

  Serial.println("\n--- Load Cell & Torque ---");
  Serial.println("readLoad                - Read load (kg), force (N), torque (Nm)");
  Serial.println("readTorque              - Read torque only (Nm)");
  Serial.println("debugLoad               - Toggle raw load cell output (every 300ms)");
  Serial.println("tare                    - Zero the load cell");
  Serial.println("calibrate               - Run interactive load cell calibration");

  Serial.println("\n--- Sensors & Debug ---");
  Serial.println("readSensor [C]          - Read AS5600 angle & RPM (C=continuous)");
  Serial.println("debug                   - Show raw sensor/motor values");

  Serial.println("\n--- Motor Power ---");
  Serial.println("enable                  - Enable all motor drivers");
  Serial.println("disable                 - Disable all motor drivers");
  Serial.println("stop                    - Emergency stop all motors");
}

// ==================== Helper Functions ====================

void displayStatus() {
  Serial.println("--- Status ---");
  Serial.print("Test Motor: Target=");
  Serial.print(targetRPM, 1);
  Serial.print(" RPM | Measured=");
  Serial.print(getRPM(), 1);
  Serial.println(" RPM");

  Serial.print("Brake Motor: Position=");
  Serial.print(brakePosition);
  Serial.println(" steps");

  Serial.print("Torque: ");
  Serial.print(getTorque(), 4);
  Serial.println(" Nm");
  Serial.println();
}

float getTorque() {
  if (LoadCell.is_ready()) {
    return LoadCell.get_units(1) * GRAVITY * ARM_LENGTH_METERS;
  }
  return 0.0f;
}

void calibrate() {
  Serial.println("\n\nLOAD CELL CALIBRATION");
  Serial.println("=====================");
  Serial.println("1. Remove all weight from the load cell.");
  while (Serial.available()) Serial.read();
  Serial.println("2. Type 'ok' or press Enter to begin.");
  while (Serial.available() == 0);
  while (Serial.read() != '\n');

  Serial.println("Determining zero offset...");
  LoadCell.tare(20);
  long offset = LoadCell.get_offset();
  Serial.print("OFFSET: ");
  Serial.println(offset);
  Serial.println();

  Serial.println("3. Place a known weight on the load cell.");
  while (Serial.available()) Serial.read();
  Serial.println("4. Enter the weight in kilograms (e.g., 0.5) and press Enter.");
  while (Serial.available() == 0);
  float known_weight_kg = Serial.parseFloat();
  Serial.print("Using known weight (kg): ");
  Serial.println(known_weight_kg, 4);

  LoadCell.calibrate_scale(known_weight_kg, 20);
  float scale = LoadCell.get_scale();
  Serial.print("SCALE:  ");
  Serial.println(scale, 15);

  Serial.println("\n--- CALIBRATION COMPLETE ---");
  Serial.println("Please update your setup() function with these values:");
  Serial.print("LoadCell.set_offset(");
  Serial.print(offset);
  Serial.print("L); \nLoadCell.set_scale(");
  Serial.print(scale, 15);
  Serial.print("f);\n\n");
}

// ==================== Automated Test Functions ====================

void handleRunTest(SerialCommander* c) {
  if (testRunning) {
    Serial.println("ERROR: Test already running. Use 'abortTest' to cancel.");
    return;
  }

  float rpm = c->getFloat(0);
  if (rpm <= 0) {
    Serial.println("Usage: runTest <rpm>");
    Serial.println("Example: runTest 500");
    return;
  }

  // Initialize test
  testTargetRPM = rpm;
  testMaxTorque = 0;
  testTorqueAtStall = 0;
  testStallStartTime = 0;
  testLastBrakeTime = millis();
  testRunning = true;

  // Return brake to home first
  brakeMotor.moveTo(0);
  brakePosition = 0;

  // Start motor at target RPM (open-loop)
  setMotorRPM(rpm);

  Serial.println("\n=== AUTOMATED TEST STARTED ===");
  Serial.print("Target RPM: ");
  Serial.println(rpm);
  Serial.print("Stall threshold: ");
  Serial.print(testStallThreshold * 100, 0);
  Serial.println("% of target");
  Serial.println("Waiting for motor to stabilize...");
  Serial.println("DATA:RPM,Torque(Nm),BrakePos");
}

void handleAbortTest(SerialCommander* c) {
  if (!testRunning) {
    Serial.println("No test running.");
    return;
  }

  testRunning = false;
  stopMotor();
  brakeMotor.moveTo(0);
  brakePosition = 0;

  Serial.println("\n=== TEST ABORTED ===");
  Serial.print("Max torque recorded: ");
  Serial.print(testMaxTorque, 4);
  Serial.println(" Nm");
}

void runTestStep() {
  float measuredRPM = getRPM();
  float currentTorque = getTorque();
  unsigned long now = millis();

  // Wait for brake motor to finish moving
  if (brakeMotor.isRunning()) {
    return;
  }

  // Check if speed is within acceptable range
  float rpmRatio = measuredRPM / testTargetRPM;
  bool speedOK = (rpmRatio >= testStallThreshold);

  // Track maximum torque while maintaining speed
  if (speedOK && currentTorque > testMaxTorque) {
    testMaxTorque = currentTorque;
  }

  // Stall detection: RPM below threshold for sustained period
  if (!speedOK) {
    if (testStallStartTime == 0) {
      testStallStartTime = now;
    } else if (now - testStallStartTime >= testStallDuration) {
      // STALL CONFIRMED
      testTorqueAtStall = currentTorque;
      testRunning = false;

      Serial.println("\n=== STALL DETECTED ===");
      Serial.print("RESULT:maxTorque=");
      Serial.print(testMaxTorque, 4);
      Serial.print(",stallTorque=");
      Serial.print(testTorqueAtStall, 4);
      Serial.print(",targetRPM=");
      Serial.print(testTargetRPM, 1);
      Serial.print(",stallRPM=");
      Serial.println(measuredRPM, 1);

      Serial.println("\n--- Test Complete ---");
      Serial.print("Maximum torque at ");
      Serial.print(testTargetRPM, 0);
      Serial.print(" RPM: ");
      Serial.print(testMaxTorque, 4);
      Serial.println(" Nm");

      // Stop motor and return brake
      stopMotor();
      brakeMotor.moveTo(0);
      brakePosition = 0;
      return;
    }
  } else {
    testStallStartTime = 0; // Reset stall timer if speed recovered
  }

  // Apply brake incrementally
  if (now - testLastBrakeTime >= testBrakeInterval) {
    brakeMotor.move(testBrakeIncrement);
    brakePosition += testBrakeIncrement;
    testLastBrakeTime = now;

    // Output data point for logging
    Serial.print("DATA:");
    Serial.print(measuredRPM, 1);
    Serial.print(",");
    Serial.print(currentTorque, 4);
    Serial.print(",");
    Serial.println(brakePosition);
  }
}
