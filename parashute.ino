#include <Wire.h>
#include <math.h>

const int MPU = 0x68;
const int EEPROM = 0x50;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int led_pin = 10;
int led_buildin = 13;
int detonator_pin = 3;
int detonatorActive = false;
int FALL_DETECTED = 0;
unsigned long fallStartTime = 0;  // To track when fall condition starts
bool fallConditionMet = false;    // To track if we're in a potential fall

// Time to trun off the detonation after the fall detection
const int TIME_TURN_OFF_MS = 2000;
// Time to trigger the fall detection detection
const int TIME_FALL_MS = 230;

void setup() {
  pinMode(led_pin, OUTPUT);
  pinMode(detonator_pin, OUTPUT);
  digitalWrite(detonator_pin, LOW);
  Wire.begin();
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
    }
  } else if (ACC_MAGNITUDE > FALL_TRESHOLD_VALUE) {
    fallConditionMet = false;  // Reset the condition tracker, but not FALL_DETECTED
  }

  // Control the LED based on FALL_DETECTED
  digitalWrite(led_pin, FALL_DETECTED);
  //digitalWrite(detonator_pin, detonatorActive);

  // Deactivate the detonator after some time
  if (detonatorActive == true && millis() - fallStartTime >= TIME_TURN_OFF_MS + TIME_FALL_MS) {
    detonatorActive = false;
  }

  // Track max acceleration values (unchanged)
  int max_measured_acc_X = 0;
  int max_measured_acc_Y = 0;
  int max_measured_acc_Z = 0;

  if (AcX > max_measured_acc_X) {
    max_measured_acc_X = AcX;
  }
  if (AcY > max_measured_acc_Y) {
    max_measured_acc_Y = AcY;
  }
  if (AcZ > max_measured_acc_Z) {
    max_measured_acc_Z = AcZ;
  }

  // Print time and sensor data
  unsigned long currentTime = millis();
  Serial.print(currentTime);
  Serial.print(" ms | ");

  Serial.print(AcX); Serial.print(" | "); Serial.print(AcY); Serial.print(" | "); Serial.print(AcZ);
  Serial.print("      "); Serial.print(ACC_MAGNITUDE); Serial.print("\n");

  // Store data
  

  delay(50);
}
