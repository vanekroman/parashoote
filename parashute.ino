#include <Wire.h>
#include <math.h>

const int MPU = 0x68;
const int EEPROM = 0x50;  // AT24CS64 EEPROM address
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int led_pin = 10;
int led_buildin = 13;
int detonator_pin = 3;
int detonatorActive = false;
int FALL_DETECTED = 0;
unsigned long fallStartTime = 0;  // To track when fall condition starts
bool fallConditionMet = false;    // To track if we're in a potential fall
// Time to turn off the detonation after the fall detection
const int TIME_TURN_OFF_MS = 2000;
// Time to trigger the fall detection detection
const int TIME_FALL_MS = 230;

// EEPROM storage variables
const int EEPROM_SIZE = 8192;     // 64 Kbit = 8 KB
const int DATA_START_ADDR = 0;    // Where to start storing data
int eepromWriteAddr = DATA_START_ADDR;
bool dataStored = false;          // Flag to indicate if fall data has been stored
const int MAX_RECORDS = (8192 / 6) - 100;       // Maximum number of acceleration records to store

// Command parsing
String inputCommand = "";
bool commandComplete = false;

void setup() {
  pinMode(led_pin, OUTPUT);
  pinMode(detonator_pin, OUTPUT);
  digitalWrite(detonator_pin, LOW);
  Wire.begin();
  
  // Initialize MPU6050
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  
  Serial.begin(9600);
  digitalWrite(led_pin, HIGH);
  digitalWrite(led_buildin, HIGH);
  delay(2000);
  digitalWrite(led_pin, LOW);
  digitalWrite(led_buildin, LOW);
  
  Serial.println("Fall detection system initialized");
  Serial.println("Available commands:");
  Serial.println("- READ_DATA: Read the stored fall acceleration data");
  Serial.println("- CLEAR_DATA: Clear all stored data from EEPROM");
}

void loop() {
  int FALL_TRESHOLD_VALUE = 5000;  // Raw units (adjust as needed)
  
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 12, true);
  AcX = Wire.read() << 8 | Wire.read();
  AcY = Wire.read() << 8 | Wire.read();
  AcZ = Wire.read() << 8 | Wire.read();
  float ACC_MAGNITUDE = sqrt(pow(AcX, 2) + pow(AcY, 2) + pow(AcZ, 2));
  
  // Free-fall detection with 250ms duration check
  if (ACC_MAGNITUDE <= FALL_TRESHOLD_VALUE && FALL_DETECTED == 0) {  // Only check if fall not yet detected
    if (!fallConditionMet) {
      fallStartTime = millis();  // Record the start time of the potential fall
      fallConditionMet = true;
    }
    // Check if the condition has persisted for 250ms
    if (millis() - fallStartTime >= TIME_FALL_MS) {
      FALL_DETECTED = 1;
      detonatorActive = 1;
      Serial.println("FALL DETECTED");
      
      digitalWrite(led_pin, FALL_DETECTED);

      // Store data to EEPROM if not already stored
      if (!dataStored) {
        storeAccelerationData();
        dataStored = true;
      }
    }
  } else if (ACC_MAGNITUDE > FALL_TRESHOLD_VALUE) {
    fallConditionMet = false;  // Reset the condition tracker, but not FALL_DETECTED
  }
  
  // Control the LED based on FALL_DETECTED
  if (FALL_DETECTED && dataStored) {
      digitalWrite(led_pin, HIGH);
      delay(500);
      digitalWrite(led_pin, LOW);
      delay(500);
  }
  //digitalWrite(detonator_pin, detonatorActive);
  
  // Deactivate the detonator after some time
  if (detonatorActive == true && millis() - fallStartTime >= TIME_TURN_OFF_MS + TIME_FALL_MS) {
    detonatorActive = false;
  }
  
  // Track max acceleration values (unchanged)
  //int max_measured_acc_X = 0;
  //int max_measured_acc_Y = 0;
  //int max_measured_acc_Z = 0;
  //if (AcX > max_measured_acc_X) {
  //  max_measured_acc_X = AcX;
  //}
  //if (AcY > max_measured_acc_Y) {
  //  max_measured_acc_Y = AcY;
  //}
  //if (AcZ > max_measured_acc_Z) {
  //  max_measured_acc_Z = AcZ;
  //}
  
  // Print time and sensor data
  if (FALL_DETECTED == false) {
    unsigned long currentTime = millis();
    Serial.print(currentTime);
    Serial.print(" ms | ");
    Serial.print(AcX); Serial.print(" | "); Serial.print(AcY); Serial.print(" | "); Serial.print(AcZ);
    Serial.print("      "); Serial.print(ACC_MAGNITUDE); Serial.print("\n");
  }
  
  // Check for serial commands
  readSerialCommand();
  processCommand();
  
  delay(1);
}

// Function to store acceleration data to EEPROM
void storeAccelerationData() {
  Serial.println("Storing fall data to EEPROM...");
  
  // Reset EEPROM write address to start
  eepromWriteAddr = DATA_START_ADDR;
  
  // Store a header with timestamp and record count
  unsigned long fallTime = millis();
  writeEEPROM(eepromWriteAddr, (fallTime >> 24) & 0xFF); eepromWriteAddr++;
  writeEEPROM(eepromWriteAddr, (fallTime >> 16) & 0xFF); eepromWriteAddr++;
  writeEEPROM(eepromWriteAddr, (fallTime >> 8) & 0xFF); eepromWriteAddr++;
  writeEEPROM(eepromWriteAddr, fallTime & 0xFF); eepromWriteAddr++;
  writeEEPROM(eepromWriteAddr, MAX_RECORDS); eepromWriteAddr++;
  
  // Collect data for a few samples
  for (int i = 0; i < MAX_RECORDS; i++) {
    // Get new accelerometer data
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 12, true);
    AcX = Wire.read() << 8 | Wire.read();
    AcY = Wire.read() << 8 | Wire.read();
    AcZ = Wire.read() << 8 | Wire.read();
    
    // Store X acceleration (2 bytes)
    writeEEPROM(eepromWriteAddr, (AcX >> 8) & 0xFF); eepromWriteAddr++;
    writeEEPROM(eepromWriteAddr, AcX & 0xFF); eepromWriteAddr++;
    
    // Store Y acceleration (2 bytes)
    writeEEPROM(eepromWriteAddr, (AcY >> 8) & 0xFF); eepromWriteAddr++;
    writeEEPROM(eepromWriteAddr, AcY & 0xFF); eepromWriteAddr++;
    
    // Store Z acceleration (2 bytes)
    writeEEPROM(eepromWriteAddr, (AcZ >> 8) & 0xFF); eepromWriteAddr++;
    writeEEPROM(eepromWriteAddr, AcZ & 0xFF); eepromWriteAddr++;
    
    // Brief delay between samples
    delay(10);
  }
  
  Serial.println("Data stored successfully!");
}

// Function to write a byte to EEPROM
void writeEEPROM(int address, byte data) {
  Wire.beginTransmission(EEPROM);
  Wire.write(address >> 8);    // High byte of address (MSB)
  Wire.write(address & 0xFF);  // Low byte of address (LSB)
  Wire.write(data);
  Wire.endTransmission();
  delay(5);  // Small delay for write cycle to complete
}

// Function to read a byte from EEPROM
byte readEEPROM(int address) {
  Wire.beginTransmission(EEPROM);
  Wire.write(address >> 8);    // High byte of address (MSB)
  Wire.write(address & 0xFF);  // Low byte of address (LSB)
  Wire.endTransmission();
  
  Wire.requestFrom(EEPROM, 1);
  if (Wire.available()) {
    return Wire.read();
  }
  return 0;
}

// Function to read serial commands
void readSerialCommand() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();
    
    if (inChar == '\n' || inChar == '\r') {
      commandComplete = true;
    } else {
      inputCommand += inChar;
    }
  }
}

// Function to process commands
void processCommand() {
  if (commandComplete) {
    inputCommand.trim();  // Remove leading/trailing whitespace
    
    if (inputCommand.equals("READ_DATA")) {
      readStoredData();
    }
    else if (inputCommand.equals("CLEAR_DATA")) {
      clearStoredData();
    }
    
    // Reset command variables
    inputCommand = "";
    commandComplete = false;
  }
}

// Function to read and display stored data
void readStoredData() {
  Serial.println("Reading fall data from EEPROM:");
  
  // Read header
  unsigned long fallTime = 0;
  fallTime |= ((unsigned long)readEEPROM(DATA_START_ADDR) << 24);
  fallTime |= ((unsigned long)readEEPROM(DATA_START_ADDR + 1) << 16);
  fallTime |= ((unsigned long)readEEPROM(DATA_START_ADDR + 2) << 8);
  fallTime |= readEEPROM(DATA_START_ADDR + 3);
  
  int recordCount = readEEPROM(DATA_START_ADDR + 4);
  
  Serial.print("Fall detected at: ");
  Serial.print(fallTime);
  Serial.println(" ms");
  Serial.print("Number of records: ");
  Serial.println(recordCount);
  
  int dataAddr = DATA_START_ADDR + 5;  // Start after header
  
  Serial.println("Index,AccX,AccY,AccZ");
  
  for (int i = 0; i < recordCount; i++) {
    // Read X acceleration (2 bytes)
    int16_t storedAcX = ((int16_t)readEEPROM(dataAddr) << 8) | readEEPROM(dataAddr + 1);
    dataAddr += 2;
    
    // Read Y acceleration (2 bytes)
    int16_t storedAcY = ((int16_t)readEEPROM(dataAddr) << 8) | readEEPROM(dataAddr + 1);
    dataAddr += 2;
    
    // Read Z acceleration (2 bytes)
    int16_t storedAcZ = ((int16_t)readEEPROM(dataAddr) << 8) | readEEPROM(dataAddr + 1);
    dataAddr += 2;
    
    Serial.print(i);
    Serial.print(",");
    Serial.print(storedAcX);
    Serial.print(",");
    Serial.print(storedAcY);
    Serial.print(",");
    Serial.println(storedAcZ);
  }
  
  Serial.println("End of data");
}

// Function to clear stored data
void clearStoredData() {
  Serial.println("Clearing stored data...");
  
  // Write zeros to the header
  for (int i = 0; i < 5; i++) {
    writeEEPROM(DATA_START_ADDR + i, 0);
  }
  
  dataStored = false;
  Serial.println("Data cleared successfully!");
}
