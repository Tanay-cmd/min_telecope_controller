#include <AccelStepper.h>
#include <EEPROM.h>

#define AZ_PUL_PIN 22
#define AZ_DIR_PIN 23

#define ALT_PUL_PIN 24
#define ALT_DIR_PIN 25

AccelStepper stepperAz(1, AZ_PUL_PIN, AZ_DIR_PIN);
AccelStepper stepperAlt(1, ALT_PUL_PIN, ALT_DIR_PIN);

// EEPROM Address Map
const int ADDR_AZ = 0;       // 4 bytes (long)
const int ADDR_ALT = 4;      // 4 bytes (long)
const int ADDR_REF_AZ = 8;   // 4 bytes (float)
const int ADDR_REF_ALT = 12; // 4 bytes (float)
const int ADDR_SIG = 16;     // 1 byte (byte)
const byte EEPROM_SIG = 0x55;

long lastSavedAz = 0;
long lastSavedAlt = 0;
unsigned long lastMotionTime = 0;
bool pendingSave = false;
const unsigned long saveDelay = 2000; // 2 seconds of idleness before EEPROM write

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
  
  // Load saved positions and references from EEPROM
  byte sig = EEPROM.read(ADDR_SIG);
  long savedAz = 0;
  long savedAlt = 0;
  float refAz = 0.0;
  float refAlt = 90.0;
  
  if (sig == EEPROM_SIG) {
    EEPROM.get(ADDR_AZ, savedAz);
    EEPROM.get(ADDR_ALT, savedAlt);
  } else {
    // Brand new EEPROM, write defaults
    EEPROM.put(ADDR_AZ, savedAz);
    EEPROM.put(ADDR_ALT, savedAlt);
    EEPROM.put(ADDR_REF_AZ, refAz);
    EEPROM.put(ADDR_REF_ALT, refAlt);
    EEPROM.write(ADDR_SIG, EEPROM_SIG);
  }
  
  stepperAz.setCurrentPosition(savedAz);
  stepperAlt.setCurrentPosition(savedAlt);
  lastSavedAz = savedAz;
  lastSavedAlt = savedAlt;
  
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
    } else if (inputString.startsWith("SET_REF")) {
      // Command format: SET_REF <ref_az> <ref_alt>
      int space1 = inputString.indexOf(' ');
      if (space1 != -1) {
        int space2 = inputString.indexOf(' ', space1 + 1);
        if (space2 != -1) {
          float refAz = inputString.substring(space1 + 1, space2).toFloat();
          float refAlt = inputString.substring(space2 + 1).toFloat();
          
          EEPROM.put(ADDR_REF_AZ, refAz);
          EEPROM.put(ADDR_REF_ALT, refAlt);
          
          Serial.print("ACK SET_REF Az:");
          Serial.print(refAz);
          Serial.print(" Alt:");
          Serial.println(refAlt);
        }
      }
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
          
          // Save immediately to EEPROM
          EEPROM.put(ADDR_AZ, currentAz);
          EEPROM.put(ADDR_ALT, currentAlt);
          lastSavedAz = currentAz;
          lastSavedAlt = currentAlt;
          pendingSave = false;
          
          Serial.print("ACK SET Az:");
          Serial.print(currentAz);
          Serial.print(" Alt:");
          Serial.println(currentAlt);
        }
      }
    } else if (inputString.startsWith("GET_REF")) {
      // Command format: GET_REF
      float refAz, refAlt;
      EEPROM.get(ADDR_REF_AZ, refAz);
      EEPROM.get(ADDR_REF_ALT, refAlt);
      
      Serial.print("REF ");
      Serial.print(refAz);
      Serial.print(" ");
      Serial.println(refAlt);
    }
    
    // Reset buffer
    inputString = "";
    stringComplete = false;
  }
  
  // EEPROM Inactivity Save Logic
  if (stepperAz.distanceToGo() != 0 || stepperAlt.distanceToGo() != 0) {
    pendingSave = true;
    lastMotionTime = millis();
  } else if (pendingSave && (millis() - lastMotionTime >= saveDelay)) {
    long curAz = stepperAz.currentPosition();
    long curAlt = stepperAlt.currentPosition();
    
    if (curAz != lastSavedAz || curAlt != lastSavedAlt) {
      EEPROM.put(ADDR_AZ, curAz);
      EEPROM.put(ADDR_ALT, curAlt);
      lastSavedAz = curAz;
      lastSavedAlt = curAlt;
      Serial.print("INFO Saved positions to EEPROM: Az:");
      Serial.print(curAz);
      Serial.print(" Alt:");
      Serial.println(curAlt);
    }
    pendingSave = false;
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
