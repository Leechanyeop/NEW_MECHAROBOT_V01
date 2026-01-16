/*
 * ESP32 Serial Communication Example
 * This sketch demonstrates how to receive and parse serial data.
 * 
 * Commands supported:
 * 1. String commands: "CMD:VALUE" (e.g., LED:ON)
 * 2. Comma-separated values: "VAL1,VAL2,VAL3" (e.g., 100,255,128)
 */

void setup() {
  // ESP32 typically uses 115200 for stable serial communication
  Serial.begin(115200);
  while (!Serial) {
    ; // Wait for serial port to connect (needed for some boards)
  }
  
  Serial.println("\n--- ESP32 Serial Receiver Started ---");
  Serial.println("Send commands like 'LED:ON' or '100,200,300'");
}

void loop() {
  // Check if data is available to read
  if (Serial.available() > 0) {
    // Read the incoming string until a newline character
    String inputString = Serial.readStringUntil('\n');
    inputString.trim(); // Remove leading/trailing whitespace

    if (inputString.length() > 0) {
      Serial.print("Received: ");
      Serial.println(inputString);

      // --- Option 1: Parse "CMD:VALUE" format ---
      if (inputString.indexOf(':') != -1) {
        int colonIndex = inputString.indexOf(':');
        String command = inputString.substring(0, colonIndex);
        String value = inputString.substring(colonIndex + 1);

        Serial.print("  [Command Mode] CMD: ");
        Serial.print(command);
        Serial.print(", VAL: ");
        Serial.println(value);

        // Logic based on command
        if (command == "LED") {
          if (value == "ON") {
            // digitalWrite(LED_PIN, HIGH);
            Serial.println("  >> LED turned ON");
          } else {
            // digitalWrite(LED_PIN, LOW);
            Serial.println("  >> LED turned OFF");
          }
        }
      } 
      // --- Option 2: Parse Comma-separated values (CSV) ---
      else if (inputString.indexOf(',') != -1) {
        Serial.println("  [CSV Mode] Values:");
        
        int startPos = 0;
        int commaIndex = inputString.indexOf(',');
        int count = 0;

        while (commaIndex != -1) {
          String part = inputString.substring(startPos, commaIndex);
          Serial.print("    Value ");
          Serial.print(count++);
          Serial.print(": ");
          Serial.println(part.toInt()); // Convert to integer if needed

          startPos = commaIndex + 1;
          commaIndex = inputString.indexOf(',', startPos);
        }
        // Last part
        String lastPart = inputString.substring(startPos);
        Serial.print("    Value ");
        Serial.print(count);
        Serial.print(": ");
        Serial.println(lastPart.toInt());
      }
      // --- Option 3: Handle single value or string ---
      else {
        Serial.println("  [Plain Text Mode]");
      }
    }
  }
}
