# Serial Communication Solutions for Arduino

This folder contains two approaches for handling serial commands on Arduino.

## Approach 1: Custom SerialCommander Class (Recommended for Students)

**Files:**
- `SerialCommander.h` - The complete implementation (students don't need to look at this)
- `SerialCom.ino` - Clean example showing how to use it

**Advantages:**
- All complexity is hidden in the .h file
- Super simple to add new commands
- No external dependencies
- Students only see clean, readable code
- Just 69 lines vs original 106 lines!

**How to add a new command:**
```cpp
// 1. Write the handler function
void handleMyCommand(SerialCommander* c) {
  int value = c->getInt(0);  // Get first parameter
  // Do something with value
}

// 2. Register it in setup()
void setup() {
  // ...
  cmd.addCommand("myCommand", handleMyCommand);
}
```

**Example usage in Serial Monitor:**
```
run
setSpeed 100
test 42 3.14 true
help
```

## Approach 2: CmdParser Library

**Files:**
- `../CmdParserExample/CmdParserExample.ino`

**Advantages:**
- Well-maintained library
- Available in Arduino Library Manager
- Good documentation
- More features if needed later

**Installation:**
1. Open Arduino IDE
2. Go to Tools → Manage Libraries
3. Search for "CmdParser"
4. Click Install

**Disadvantages:**
- Requires installing a library
- Students see more complex parsing code
- More to explain/teach

## Which Should I Use for Teaching?

**For beginners → Use SerialCommander.h**
- Students just copy `SerialCommander.h` to their project folder
- They never need to look inside it
- The `.ino` file is crystal clear
- No library installation needed

**For advanced students → Use CmdParser**
- Good practice using external libraries
- Industry-standard approach
- More powerful features available

## Comparison: Old vs New Code

**OLD (106 lines with complex parsing):**
```cpp
void ProcessSerialData() {
  if (newData == false) return;
  newData = false;
  strcpy(tempChars, receivedChars);
  char* strtokIndx;
  strtokIndx = strtok(tempChars, " ");
  // ... 20+ more lines of parsing logic
}
```

**NEW (69 lines, clean and simple):**
```cpp
void handleSetSpeed(SerialCommander* c) {
  int speed = c->getInt(0);
  Serial.println(speed);
}

void setup() {
  cmd.addCommand("setSpeed", handleSetSpeed);
}
```

## Features of SerialCommander

- **Automatic parsing** - Handles all tokenization
- **Type conversion** - `getInt()`, `getFloat()`, `getBool()`, `getString()`
- **Error handling** - Unknown commands automatically show help
- **Help system** - Built-in `printHelp()` function
- **Callback system** - Easy to organize code
- **No dynamic memory** - Safe for embedded systems
- **Efficient** - Minimal overhead
