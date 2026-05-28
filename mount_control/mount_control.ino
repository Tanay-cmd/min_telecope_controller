#include <AccelStepper.h>

#define AZ_PUL_PIN 22
#define AZ_DIR_PIN 23

#define ALT_PUL_PIN 24
#define ALT_DIR_PIN 25

AccelStepper stepperAz(1, AZ_PUL_PIN, AZ_DIR_PIN);
AccelStepper stepperAlt(1, ALT_PUL_PIN, ALT_DIR_PIN);

unsigned long lastTelemetryTime = 0;
const unsigned long telemetryInterval = 500; // ms

String inputString = "";
bool stringComplete = false;

void setup() {
  Serial.begin(9600);
  while (!Serial) { ; }
  
  Serial.println("--- Alt-Az Mount Controller Initialized ---");
  
  // Set default speed/accel configurations
  stepperAz.setMaxSpeed(2000.0);
  stepperAz.setAcceleration(1000.0);
  
  stepperAlt.setMaxSpeed(2000.0);
  stepperAlt.setAcceleration(1000.0);
  
  inputString.reserve(64);
}

void loop() {
  // Always call run() for both steppers to execute step pulses
  stepperAz.run();
  stepperAlt.run();
  
  // Read Serial input
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      stringComplete = true;
    } else if (inChar != '\r') { // Ignore carriage returns
      inputString += inChar;
    }
  }
  
  // Parse complete command string
  if (stringComplete) {
    inputString.trim();
    if (inputString.startsWith("MOVE")) {
      // Command format: MOVE <az_steps> <alt_steps>
      int space1 = inputString.indexOf(' ');
      if (space1 != -1) {
        int space2 = inputString.indexOf(' ', space1 + 1);
        if (space2 != -1) {
          long targetAz = inputString.substring(space1 + 1, space2).toInt();
          long targetAlt = inputString.substring(space2 + 1).toInt();
          
          stepperAz.moveTo(targetAz);
          stepperAlt.moveTo(targetAlt);
          
          Serial.print("ACK MOVE Az:");
          Serial.print(targetAz);
          Serial.print(" Alt:");
          Serial.println(targetAlt);
        }
      }
    } else if (inputString.startsWith("HALT")) {
      // Command format: HALT
      stepperAz.stop();
      stepperAlt.stop();
      Serial.println("ACK HALT");
    } else if (inputString.startsWith("SET")) {
      // Command format: SET <az_steps> <alt_steps>
      int space1 = inputString.indexOf(' ');
      if (space1 != -1) {
        int space2 = inputString.indexOf(' ', space1 + 1);
        if (space2 != -1) {
          long currentAz = inputString.substring(space1 + 1, space2).toInt();
          long currentAlt = inputString.substring(space2 + 1).toInt();
          
          stepperAz.setCurrentPosition(currentAz);
          stepperAlt.setCurrentPosition(currentAlt);
          
          Serial.print("ACK SET Az:");
          Serial.print(currentAz);
          Serial.print(" Alt:");
          Serial.println(currentAlt);
        }
      }
    }
    
    // Reset buffer
    inputString = "";
    stringComplete = false;
  }
  
  // Periodic Telemetry Output
  unsigned long currentTime = millis();
  if (currentTime - lastTelemetryTime >= telemetryInterval) {
    lastTelemetryTime = currentTime;
    Serial.print("TELEMETRY ");
    Serial.print(stepperAz.currentPosition());
    Serial.print(" ");
    Serial.println(stepperAlt.currentPosition());
  }
}
