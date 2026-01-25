/*
 * CmdParser Library Example
 *
 * This example demonstrates using the CmdParser library for serial commands.
 * Install via: Arduino Library Manager -> Search "CmdParser" -> Install
 *
 * Commands:
 * - run
 * - setSpeed <int>
 * - test <int> <float> <bool>
 *
 * Compare with SerialCommander (in lib/) for a zero-dependency alternative.
 */

#include <CmdParser.hpp>

void setup() {
  Serial.begin(115200);
  Serial.println("Arduino is ready (CmdParser version)");
  Serial.println("Try: run, setSpeed 100, test 42 3.14 true");
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    command.trim();

    CmdParser parser;
    parser.setOptKeyValue(false);
    parser.parseCmd((char*)command.c_str());

    if (parser.equalCommand("run")) {
      handleRun();
    }
    else if (parser.equalCommand("setSpeed")) {
      handleSetSpeed(&parser);
    }
    else if (parser.equalCommand("test")) {
      handleTest(&parser);
    }
    else if (parser.equalCommand("help")) {
      displayHelp();
    }
    else {
      Serial.println("Unknown command.");
      displayHelp();
    }
  }
}

void handleRun() {
  Serial.println("Running...");
}

void handleSetSpeed(CmdParser *parser) {
  int speed = parser->getCmdParam(1).toInt();
  Serial.print("Speed is: ");
  Serial.println(speed);
}

void handleTest(CmdParser *parser) {
  int int_val = parser->getCmdParam(1).toInt();
  float float_val = parser->getCmdParam(2).toFloat();

  String bool_str = parser->getCmdParam(3);
  bool_str.toLowerCase();
  bool bool_val = (bool_str == "true" || bool_str == "1");

  Serial.print("int_val: ");
  Serial.println(int_val);
  Serial.print("float_val: ");
  Serial.println(float_val);
  Serial.print("bool_val: ");
  Serial.println(bool_val);
}

void displayHelp() {
  Serial.println("Valid commands:");
  Serial.println("  run");
  Serial.println("  setSpeed <value>");
  Serial.println("  test <int> <float> <bool>");
  Serial.println("  help");
}
