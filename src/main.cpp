/*
 * Dyno - Stepper Motor Dynamometer
 *
 * Open-loop stepper speed control with AS5600 RPM measurement
 * and HX711 load cell for force measurement.
 *
 * Hybrid motor control:
 * - Test motor: AccelStepper (polled) - supports high step rates (>10k steps/s)
 * - Brake motor: MobaTools (interrupt-driven) - runs independently of loop()
 *
 * Load cell calibration and torque calculation is done in Python.
 * Firmware only sends raw HX711 values.
 *
 * Hardware:
 * - RAMPS 1.4 shield
 * - Test motor on X-axis (AccelStepper, polled)
 * - Brake actuator motor on Y-axis (MobaTools, interrupt-driven)
 * - AS5600 on I2C (SDA=20, SCL=21 on RAMPS)
 * - HX711 on Digital Pins (D4, D5)
 */

#define VERSION "4.0.0"

#include <Arduino.h>
#include <AccelStepper.h>
#include <MobaTools.h>
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
#define MAX_SPEED          15000
#define TEST_MOTOR_STEPS_PER_REV    (200 * 8)  // 200 full steps x 8 microsteps (TMC2226, no jumpers)
#define BRAKE_MOTOR_STEPS_PER_REV   (200 * 8)  // 200 full steps x 8 microsteps (TMC2226, no jumpers)

#define BRAKE_RAMP_LEN     200   // Steps for brake motor acceleration ramp

#define SCREW_PITCH        2     // Trapezoidal screw pitch in mm

// === GLOBAL OBJECTS ===
// Test motor: AccelStepper (polled in loop, high step rate capability)
AccelStepper testMotor(AccelStepper::DRIVER, X_STEP_PIN, X_DIR_PIN);
// Brake motor: MobaTools (interrupt-driven, won't interfere with test motor)
MoToStepper brakeMotor(BRAKE_MOTOR_STEPS_PER_REV, STEPDIR);

AS5600 as5600;
HX711 LoadCell;
SerialCommander cmd;

// === GLOBAL STATE ===
float targetRPM = 0;
float currentRPM = 0;
unsigned long lastRpmTime = 0;
long brakePosition = 0;
float lastLoadRaw = 0;  // Last raw HX711 reading

// === FORWARD DECLARATIONS ===
void updateRPM();
void updateLoad();
float getRPM();
void setMotorRPM(float rpm);
void stopMotor();
void handleSetRPM(SerialCommander* c);
void handleBrake(SerialCommander* c);
void handleBrakeApply(SerialCommander* c);
void handleBrakeHome(SerialCommander* c);
void handleStatus(SerialCommander* c);
void handleStop(SerialCommander* c);
void handleReadLoad(SerialCommander* c);
void handleHelp(SerialCommander* c);
void handleDebug(SerialCommander* c);
void handleEnable(SerialCommander* c);
void handleDisable(SerialCommander* c);
void handleReadRPM(SerialCommander* c);
void brakeMotorMove(SerialCommander* c);


void setup() {
  Serial.begin(115200);
  Wire.begin();

  // --- Initialize AS5600 ---
  as5600.begin(3);
  Serial.print("AS5600 connected: ");
  Serial.println(as5600.isConnected() ? "Yes" : "No");

  // --- Initialize Load Cell (HX711) ---
  LoadCell.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  Serial.println("HX711 initialized.");

  // --- Configure Test Motor (AccelStepper, polled) ---
  testMotor.setEnablePin(X_ENABLE_PIN);
  testMotor.setPinsInverted(false, false, true);
  testMotor.disableOutputs();
  testMotor.setMaxSpeed(MAX_SPEED);

  // --- Configure Brake Motor (MobaTools, interrupt-driven) ---
  brakeMotor.attach(Y_STEP_PIN, Y_DIR_PIN);
  brakeMotor.attachEnable(Y_ENABLE_PIN, 10, LOW);
  brakeMotor.setRampLen(BRAKE_RAMP_LEN);
  brakeMotor.setSpeedSteps(10000);  // 1000 steps/s

  // --- Register Commands ---
  cmd.begin("Dyno Ready v" VERSION);
  cmd.addCommand("setRPM", handleSetRPM);
  cmd.addCommand("brake", handleBrake);
  cmd.addCommand("brakeMove", brakeMotorMove);
  cmd.addCommand("brakeApply", handleBrakeApply);
  cmd.addCommand("brakeHome", handleBrakeHome);
  cmd.addCommand("status", handleStatus);
  cmd.addCommand("readLoad", handleReadLoad);
  cmd.addCommand("readRPM", handleReadRPM);
  cmd.addCommand("stop", handleStop);
  cmd.addCommand("help", handleHelp);
  cmd.addCommand("debug", handleDebug);
  cmd.addCommand("enable", handleEnable);
  cmd.addCommand("disable", handleDisable);

  Serial.println("Type 'help' for available commands");
}

void loop() {
  cmd.process();

  // Skip ALL interrupt-heavy sensor reads while a command is arriving.
  // Both HX711 (noInterrupts) and I2C/Wire (clock stretching) can drop
  // UART bytes at 115200 baud if they fire between bytes of a command.
  if (!cmd.isReceiving()) {
    updateRPM();
    updateLoad();
  }

  // Only the test motor needs polling - brake motor runs via interrupts
  testMotor.runSpeed();
}

// ==================== Sensor Updates (non-blocking) ====================

void updateRPM() {
  if (millis() - lastRpmTime >= 25) {
    currentRPM = -as5600.getAngularSpeed(AS5600_MODE_RPM);
    lastRpmTime = millis();
  }
}

void updateLoad() {
  // Skip HX711 read if serial data is arriving — noInterrupts() during
  // bit-banging would cause UART bytes to be dropped at 115200 baud
  if (!Serial.available() && LoadCell.is_ready()) {
    lastLoadRaw = LoadCell.read();
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
  Serial.println("OK");
}

void brakeMotorMove(SerialCommander* c) {
  long dist = c->getInt(0); // in millimeters
  int pitch = SCREW_PITCH;
  long steps = BRAKE_MOTOR_STEPS_PER_REV * dist / pitch;
  brakeMotor.doSteps(steps);
  brakePosition += steps;
  Serial.println(brakePosition);
}

void handleBrake(SerialCommander* c) {
  long steps = c->getInt(0);
  brakeMotor.doSteps(steps);
  brakePosition += steps;
  Serial.println(brakePosition);
}

void handleBrakeApply(SerialCommander* c) {
  long steps = (c->getParamCount() > 0) ? c->getInt(0) : 35000;
  brakeMotor.doSteps(steps);
  brakePosition += steps;
  Serial.println(brakePosition);
}

void handleBrakeHome(SerialCommander* c) {
  brakeMotor.writeSteps(0);
  brakePosition = 0;
  Serial.println("OK");
}

void handleStatus(SerialCommander* c) {
  Serial.print(getRPM(), 1);    Serial.print(',');
  Serial.print(targetRPM, 1);   Serial.print(',');
  Serial.print(lastLoadRaw, 0); Serial.print(',');
  Serial.print(brakePosition);  Serial.print(',');
  Serial.println(brakeMotor.moving());
}

void handleReadLoad(SerialCommander* c) {
  // Library's built-in median of 5 reads. ~500ms at 10 SPS.
  Serial.println(LoadCell.read_median(5), 0);
}

void handleReadRPM(SerialCommander* c) {
  Serial.println(getRPM(), 2);
}

void handleDebug(SerialCommander* c) {
  Serial.print("angle:"); Serial.print(as5600.readAngle());
  Serial.print(",steps/s:"); Serial.print(testMotor.speed(), 0);
  Serial.print(",rpm:"); Serial.print(getRPM(), 2);
  Serial.print(",hx_ready:"); Serial.print(LoadCell.is_ready());
  Serial.print(",hx_read:"); Serial.print(LoadCell.read(), 0);
  Serial.print(",load_raw:"); Serial.println(lastLoadRaw, 0);
}

void handleEnable(SerialCommander* c) {
  testMotor.enableOutputs();
  digitalWrite(Y_ENABLE_PIN, LOW);
  Serial.println("OK");
}

void handleDisable(SerialCommander* c) {
  stopMotor();
  brakeMotor.stop();
  testMotor.disableOutputs();
  digitalWrite(Y_ENABLE_PIN, HIGH);
  Serial.println("OK");
}

void handleStop(SerialCommander* c) {
  stopMotor();
  brakeMotor.stop();
  Serial.println("OK");
}

void handleHelp(SerialCommander* c) {
  Serial.println(F("setRPM|brake|brakeMove|brakeApply|brakeHome|status|readLoad|readRPM|debug|enable|disable|stop"));
}
