/*
 * SerialCommander - Easy Serial Command Handler for Arduino
 *
 * This class hides all the complexity of parsing serial commands,
 * making it easy to add new commands with a simple callback system.
 *
 * Usage:
 *   SerialCommander cmd;
 *
 *   void setup() {
 *     Serial.begin(115200);
 *     cmd.begin();
 *     cmd.addCommand("run", handleRun);
 *     cmd.addCommand("setSpeed", handleSetSpeed);
 *   }
 *
 *   void loop() {
 *     cmd.process();
 *   }
 */

#ifndef SERIAL_COMMANDER_H
#define SERIAL_COMMANDER_H

#include <Arduino.h>

// Maximum number of commands and parameters
#define MAX_COMMANDS 20
#define MAX_PARAMS 10
#define BUFFER_SIZE 64

class SerialCommander {
public:
  // Constructor
  SerialCommander() : commandCount(0), newData(false), cmdIndex(0) {}

  // Initialize the commander
  void begin(const char* readyMessage = "Arduino is ready") {
    Serial.println(readyMessage);
  }

  // Add a command with a callback function
  // The callback receives this SerialCommander instance
  void addCommand(const char* name, void (*callback)(SerialCommander*)) {
    if (commandCount < MAX_COMMANDS) {
      commandNames[commandCount] = name;
      commandCallbacks[commandCount] = callback;
      commandCount++;
    }
  }

  // Process incoming serial data (call this in loop())
  void process() {
    readSerial();
    if (newData) {
      parseAndExecute();
      newData = false;
    }
  }

  // Get parameter by index as different types
  int getInt(int index) {
    if (index < paramCount) {
      return atoi(params[index]);
    }
    return 0;
  }

  float getFloat(int index) {
    if (index < paramCount) {
      return atof(params[index]);
    }
    return 0.0;
  }

  bool getBool(int index) {
    if (index < paramCount) {
      if (!strncasecmp(params[index], "true", 4) || strcmp(params[index], "1") == 0) {
        return true;
      }
    }
    return false;
  }

  String getString(int index) {
    if (index < paramCount) {
      return String(params[index]);
    }
    return "";
  }

  // Get the number of parameters
  int getParamCount() {
    return paramCount;
  }

  // Print a help message with all registered commands
  void printHelp() {
    Serial.println("Available commands:");
    for (int i = 0; i < commandCount; i++) {
      Serial.print("  ");
      Serial.println(commandNames[i]);
    }
  }

private:
  // Command storage
  const char* commandNames[MAX_COMMANDS];
  void (*commandCallbacks[MAX_COMMANDS])(SerialCommander*);
  int commandCount;

  // Parameter storage
  char params[MAX_PARAMS][32];
  int paramCount;

  // Input buffer
  char buffer[BUFFER_SIZE];
  int cmdIndex;
  bool newData;

  // Read serial data into buffer
  void readSerial() {
    while (Serial.available() > 0 && !newData) {
      char received = Serial.read();

      if (received != '\n' && received != '\r') {
        if (cmdIndex < BUFFER_SIZE - 1) {
          buffer[cmdIndex++] = received;
        }
      } else if (cmdIndex > 0) {
        buffer[cmdIndex] = '\0';
        cmdIndex = 0;
        newData = true;
      }
    }
  }

  // Parse the buffer and execute the command
  void parseAndExecute() {
    // Reset parameter count
    paramCount = 0;

    // Make a copy for tokenization
    char tempBuffer[BUFFER_SIZE];
    strcpy(tempBuffer, buffer);

    // Get command name
    char* token = strtok(tempBuffer, " ");
    if (token == NULL) return;

    char command[32];
    strcpy(command, token);

    // Get parameters
    while ((token = strtok(NULL, " ")) != NULL && paramCount < MAX_PARAMS) {
      strcpy(params[paramCount], token);
      paramCount++;
    }

    // Find and execute command
    bool found = false;
    for (int i = 0; i < commandCount; i++) {
      if (strcmp(command, commandNames[i]) == 0) {
        commandCallbacks[i](this);
        found = true;
        break;
      }
    }

    if (!found) {
      Serial.print("Unknown command: ");
      Serial.println(command);
      printHelp();
    }
  }
};

#endif
