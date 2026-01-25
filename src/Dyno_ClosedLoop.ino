/*
 * Dyno - Closed-Loop Speed & Torque Measurement Version
 *
 * This version uses AS5600 feedback to maintain constant motor speed
 * and an HX711 load cell to measure torque.
 *
 * Features:
 * - PID-controlled constant RPM on test motor (X-axis)
 * - AS5600 magnetic sensor for accurate speed measurement
 * - HX711-based load cell for torque measurement
 * - SerialCommander for easy control
 * - Manual brake motor control (Y-axis)
 *
 * Hardware:
 * - RAMPS 1.4 shield
 * - Test motor on X-axis
 * - Brake actuator motor on Y-axis
 * - AS5600 on I2C (SDA=20, SCL=21 on RAMPS)
 * - HX711 on Digital Pins (D4, D5)
 */

#include <AccelStepper.h>
#include <Wire.h>
#include "AS5600.h"
#include <HX711.h>
#include "SpeedController.h"
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

#define TEST_MOTOR_STEPS_PER_REV    200
#define BRAKE_MOTOR_STEPS_PER_REV   200

#define ARM_LENGTH_METERS  0.1f   // IMPORTANT: Measure and update this value!
#define GRAVITY            9.80665f

// === GLOBAL OBJECTS ===
AccelStepper testMotor = AccelStepper(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
AccelStepper brakeMotor = AccelStepper(AccelStepper::DRIVER, Y_STEP_PIN, Y_DIR_PIN);
AS5600 as5600;
HX711 LoadCell;
SpeedController speedController(&testMotor, &as5600, TEST_MOTOR_STEPS_PER_REV);
SerialCommander cmd;

// === GLOBAL STATE ===
unsigned long lastStatusTime = 0;
bool autoStatus = false;
long brakePosition = 0;

// === FORWARD DECLARATIONS ===
void displayStatus();
float getTorque();
void calibrate();
void handleSetRPM(SerialCommander* c);
void handleBrake(SerialCommander* c);
void handleBrakeApply(SerialCommander* c);
void handleBrakeRelease(SerialCommander* c);
void handleBrakeHome(SerialCommander* c);
void handleStatus(SerialCommander* c);
void handleTune(SerialCommander* c);
void handleStop(SerialCommander* c);
void handleAutoStatus(SerialCommander* c);
void handleReadTorque(SerialCommander* c);
void handleTare(SerialCommander* c);
void handleCalibrate(SerialCommander* c);
void handleHelp(SerialCommander* c);


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
  testMotor.setPinsInverted(false, false, true);
  testMotor.enableOutputs();
  testMotor.setMaxSpeed(MAX_SPEED);

  brakeMotor.setEnablePin(Y_ENABLE_PIN);
  brakeMotor.setPinsInverted(false, false, true);
  brakeMotor.enableOutputs();
  brakeMotor.setMaxSpeed(MAX_SPEED);
  brakeMotor.setAcceleration(ACCELERATION);

  // --- Initialize Controllers ---
  speedController.begin();

  // --- Register Commands ---
  cmd.begin("Dyno Ready - Closed-Loop Control v1.2");
  cmd.addCommand("setRPM", handleSetRPM);
  cmd.addCommand("brake", handleBrake);
  cmd.addCommand("brakeApply", handleBrakeApply);
  cmd.addCommand("brakeRelease", handleBrakeRelease);
  cmd.addCommand("brakeHome", handleBrakeHome);
  cmd.addCommand("status", handleStatus);
  cmd.addCommand("autoStatus", handleAutoStatus);
  cmd.addCommand("tune", handleTune);
  cmd.addCommand("readTorque", handleReadTorque);
  cmd.addCommand("tare", handleTare);
  cmd.addCommand("calibrate", handleCalibrate);
  cmd.addCommand("stop", handleStop);
  cmd.addCommand("help", handleHelp);

  Serial.println("Type 'help' for available commands");
}

void loop() {
  cmd.process();
  speedController.update();
  brakeMotor.run();

  if (autoStatus && (millis() - lastStatusTime > 500)) {
    displayStatus();
    lastStatusTime = millis();
  }
}

// ==================== Command Handlers ====================

void handleSetRPM(SerialCommander* c) {
  float rpm = c->getFloat(0);
  speedController.setTargetRPM(rpm);
  Serial.print("Target RPM set to: ");
  Serial.println(rpm);
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

void handleTune(SerialCommander* c) {
  if (c->getParamCount() < 3) {
    Serial.println("Usage: tune <kp> <ki> <kd>"); return;
  }
  float kp = c->getFloat(0);
  float ki = c->getFloat(1);
  float kd = c->getFloat(2);
  speedController.setPID(kp, ki, kd);
  Serial.println("PID parameters updated.");
}

void handleReadTorque(SerialCommander* c) {
  Serial.print("Torque: ");
  Serial.print(getTorque(), 4);
  Serial.println(" Nm");
}

void handleTare(SerialCommander* c) {
  Serial.println("Taring load cell... do not touch.");
  LoadCell.tare(10);
  Serial.println("Tare complete.");
}

void handleCalibrate(SerialCommander* c) {
  calibrate();
}

void handleStop(SerialCommander* c) {
  speedController.setTargetRPM(0);
  brakeMotor.stop();
  Serial.println("EMERGENCY STOP - All motors stopped");
}

void handleHelp(SerialCommander* c) {
  Serial.println("\n=== Dyno Commands ===");
  Serial.println("\n--- Test Motor & PID ---");
  Serial.println("setRPM <value>          - Set target RPM (0 to disable)");
  Serial.println("status                  - Display current system status");
  Serial.println("autoStatus [true/false] - Toggle auto status display");
  Serial.println("tune <kp> <ki> <kd>     - Tune PID parameters");

  Serial.println("\n--- Brake Control ---");
  Serial.println("brake <steps>           - Move brake motor (+apply / -release)");
  Serial.println("brakeApply [steps]      - Apply brake (default: 500)");
  Serial.println("brakeRelease [steps]    - Release brake (default: 500)");
  Serial.println("brakeHome               - Return brake to home position (0)");

  Serial.println("\n--- Torque Measurement ---");
  Serial.println("readTorque              - Read current torque from load cell");
  Serial.println("tare                    - Zero the load cell");
  Serial.println("calibrate               - Run interactive load cell calibration");

  Serial.println("\n--- Safety ---");
  Serial.println("stop                    - Emergency stop all motors");
}

// ==================== Helper Functions ====================

void displayStatus() {
  Serial.println("--- Status ---");
  Serial.print("Test Motor: Target=");
  Serial.print(speedController.getTargetRPM(), 1);
  Serial.print(" RPM | Current=");
  Serial.print(speedController.getCurrentRPM(), 1);
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
