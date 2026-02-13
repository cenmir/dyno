/*
 * Improved Serial Communication using SerialCommander
 *
 * All the complexity is hidden in SerialCommander.h
 * Adding new commands is as simple as:
 *   1. Write a handler function
 *   2. Register it in setup() with cmd.addCommand()
 */

#include "SerialCommander.h"

// Create a SerialCommander instance
SerialCommander cmd;

void setup() {
  Serial.begin(115200);
  cmd.begin();  // Prints "Arduino is ready"

  // Register all commands with their handlers
  cmd.addCommand("run", handleRun);
  cmd.addCommand("setSpeed", handleSetSpeed);
  cmd.addCommand("test", handleTest);
  cmd.addCommand("help", handleHelp);
}

void loop() {
  // Process incoming serial commands
  cmd.process();
}

// ==================== Command Handlers ====================

void handleRun(SerialCommander* c) {
  Serial.println("Running...");
  // Add your run logic here
}

void handleSetSpeed(SerialCommander* c) {
  int speed = c->getInt(0);  // Get first parameter as int

  Serial.print("Speed is: ");
  Serial.println(speed);
  // Add your speed control logic here
}

void handleTest(SerialCommander* c) {
  int int_val = c->getInt(0);      // First parameter
  float float_val = c->getFloat(1); // Second parameter
  bool bool_val = c->getBool(2);    // Third parameter

  Serial.print("int_val: ");
  Serial.println(int_val);

  Serial.print("float_val: ");
  Serial.println(float_val);

  Serial.print("bool_val: ");
  Serial.println(bool_val);
  // Add your test logic here
}

void handleHelp(SerialCommander* c) {
  Serial.println("Valid commands are:");
  Serial.println("  run");
  Serial.println("  setSpeed <int>");
  Serial.println("  test <int> <float> <bool>");
  Serial.println("  help");
}
