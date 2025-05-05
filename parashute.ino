#include <Wire.h>
#include <math.h>
#include <TimerOne.h>

//_________ ADJUST IF NEEDED ________
// select MPU range form +- 2g to +- 16 g
//#define MPU_RANGE 0  // 2g
#define MPU_RANGE 1  // 4g
//#define MPU_RANGE 2  // 8g
//#define MPU_RANGE 3 // 16g
// select threshold value for 2g range
#define FALL_TRESHOLD (float)5000.0f
// time to trigger the fall detection detection
#define TIME_FALL_MS 230
// time to turn off the detonation after the fall detection
#define TIME_TURN_OFF_MS 100
// period of timer trigger/mpu read period in micro seconds
#define MPU_READ_PERIOD_US 10000
// blink period of sleep state
#define BLINK_PERIOD_MS 500

#define SERIAL_BAUDRATE 115200

#define DEBUG_MODE 0

// memory
#define MEMORY_PAGE_SIZE_BYTES 64
#define MEMORY_NUM_PAGES 500
#define MEMORY_SIZE_BYTES (unsigned long)(MEMORY_NUM_PAGES * MEMORY_PAGE_SIZE_BYTES)
// ram buffer for pre-storring mpu data <= MEMORY_PAGE_SIZE_BYTES / 6
// number of records in buffer - also number of records per page max 10
#define MEMORY_BUFFER_RECORDS 5

//__________  DEFINITIONS  __________
#define MPU_ADDRESS 0x68
#define EEPROM_ADDRESS 0x50

//________ MPU6050 REGISTERS ________
#define MPU_PWR_MGMT_1 0x6B
#define MPU_ACCEL_AXES_BASE 0x3B

//__________ HARDWARE SPEC __________
#define PIN_LED_TYPE OUTPUT
#define PIN_DETONATOR_TYPE OUTPUT

#define PIN_LED 10
#define PIN_DETONATOR 3
#define PIN_BUILDIN 13

//__________ THRESHOLD CALC __________
#if MPU_RANGE == 0
#define FALL_TRESHOLD_VALUE (float)(FALL_TRESHOLD)
#elif MPU_RANGE == 1
#define FALL_TRESHOLD_VALUE (float)(FALL_TRESHOLD / 2)
#elif MPU_RANGE == 2
#define FALL_TRESHOLD_VALUE (float)(FALL_TRESHOLD / 4)
#elif MPU_RANGE == 3
#define FALL_TRESHOLD_VALUE (float)(FALLFALL_TRESHOLD / 8)
#else
#error "Invalid MPU_RANGE"
#define FALL_TRESHOLD_VALUE (float)(FALL_TRESHOLD)
#endif

//__________   VARIABLES   __________
enum state_t {
  MEASURE,
  DETECT,
  DETONATE,
  DEPLOYED,
  SLEEP,
  TEST
};

#if DEBUG_MODE
enum state_t state = DETONATE;
#else
enum state_t state = MEASURE;
#endif

int16_t acc_x, acc_y, acc_z = 0;

// fall detection variables
bool is_fall_detected = false;
bool is_fall_condition_met = false;
unsigned long fall_start_time = 0;

// detonator variables
bool is_detonator_active = false;
unsigned long detonator_start_time = 0;

// serial api handling
String input_command = "";
bool is_command_complete = false;

// memory variables
struct data_t {
  int16_t x;
  int16_t y;
  int16_t z;
};

struct data_t buffer[MEMORY_BUFFER_RECORDS];
int buffer_adr = 0;
int eeprom_write_adr = 0;
// timer interrupt for acc sensor read triggered
volatile bool is_read_flag = false;

// serail func decl
void serial_read();
void process_command();

//_____________  SETUP ______________
void setup() {
  // setup of pins
  pinMode(PIN_LED, PIN_LED_TYPE);
  pinMode(PIN_DETONATOR, PIN_DETONATOR_TYPE);
  digitalWrite(PIN_DETONATOR, LOW);

  Wire.begin();

  // Reset MPU6050
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  // accelerometer range settings
  Wire.beginTransmission(MPU_ADDRESS);
  // ACCEL_CONFIG register
  Wire.write(0x1C);
  Wire.write(MPU_RANGE << 3);
  Wire.endTransmission();


  Serial.begin(SERIAL_BAUDRATE);

  Timer1.initialize(MPU_READ_PERIOD_US);

  // Print Available Commands
  Serial.println("Fall detection system initialized");
  Serial.println("Available commands:");
  Serial.println("- READ_DATA: Read the stored fall acceleration data");
  Serial.println("- CLEAR_DATA: Clear all stored data from EEPROM");

  digitalWrite(PIN_LED, HIGH);

  const int time_wait = 10000;
  for (int i = 0; i < time_wait; i++) {
    serial_read();
    process_command();
    delay(1);
  }

  digitalWrite(PIN_LED, LOW);
  delay(500);
  digitalWrite(PIN_LED, HIGH);
  delay(500);
  digitalWrite(PIN_LED, LOW);
}

void mpu_read() {
  Wire.beginTransmission(MPU_ADDRESS);
  Wire.write(MPU_ACCEL_AXES_BASE);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDRESS, 12, true);

  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
}

enum state_t detect_fall() {
  // state to retun: MEASURE / DETONATE
  enum state_t r_state = MEASURE;

  float acc = sqrt(pow(acc_x, 2) + pow(acc_y, 2) + pow(acc_z, 2));

  // Free-fall detection with TIME_FALL_MS duration check
  // Only check if fall not yet detected
  if (acc <= FALL_TRESHOLD_VALUE && !is_fall_detected) {

    if (!is_fall_condition_met) {
      // Record the start time of the potential fall
      fall_start_time = millis();
      is_fall_condition_met = true;
    }

    // Check if the condition has persisted for TIME_FALL_MS
    if (millis() - fall_start_time >= TIME_FALL_MS) {
      is_fall_detected = true;
      r_state = DETONATE;
    }

  } else if (acc > FALL_TRESHOLD_VALUE) {
    // Reset the condition tracker, but not FALL_DETECTED
    is_fall_condition_met = false;
  }

  return r_state;
}

void eeprom_write(int address, struct data_t* data, int count) {
  int size = count * sizeof(struct data_t);

  if (address < 0 || (address + size) > MEMORY_SIZE_BYTES || size > MEMORY_PAGE_SIZE_BYTES) {
    Serial.println("EEPROM write error: out of memory bounds");
    return;
  }

  Wire.beginTransmission(EEPROM_ADDRESS);
  // MSB of EEPROM address
  Wire.write(address >> 8);
  // LSB of EEPROM address
  Wire.write(address & 0xFF);

  for (int i = 0; i < count; i++) {
    Wire.write((data[i].x >> 8) & 0xFF);
    Wire.write(data[i].x & 0xFF);
    Wire.write((data[i].y >> 8) & 0xFF);
    Wire.write(data[i].y & 0xFF);
    Wire.write((data[i].z >> 8) & 0xFF);
    Wire.write(data[i].z & 0xFF);
  }

  Wire.endTransmission();
}

// Function to read from EEPROM
void eeprom_read(int address, struct data_t* buffer, int count) {
  int size = count * sizeof(struct data_t);

  if (address < 0 || (address + size) > MEMORY_SIZE_BYTES) {
    Serial.println("EEPROM read error: out of memory bounds");
    return;
  }

  Wire.beginTransmission(EEPROM_ADDRESS);
  // MSB of EEPROM address
  Wire.write(address >> 8);
  // LSB of EEPROM address
  Wire.write(address & 0xFF);
  Wire.endTransmission();

  Wire.requestFrom(EEPROM_ADDRESS, size);

  for (int i = 0; i < count; i++) {
    buffer[i].x = (Wire.read() << 8) | Wire.read();
    buffer[i].y = (Wire.read() << 8) | Wire.read();
    buffer[i].z = (Wire.read() << 8) | Wire.read();
  }
}

enum state_t memory_store() {

  int write_size = MEMORY_BUFFER_RECORDS * sizeof(struct data_t);
#if DEBUG_MODE
  Serial.print("MEMORY_STORE: write_size = ");
  Serial.println(write_size);
  Serial.print("MEMORY_STORE: eeprom_write_adr = ");
  Serial.println(eeprom_write_adr);
#endif
  // prevent out-of-bounds write
  if (eeprom_write_adr + write_size >= MEMORY_SIZE_BYTES) {
#if DEBUG_MODE
    Serial.println("JUMP to SLEEP state");
    Serial.print("MEMORY_STORE: MEMORY_SIZE_BYTES = ");
    Serial.println(MEMORY_SIZE_BYTES);
#endif
    return SLEEP;
  }
  // provent out of page write
  if (write_size > MEMORY_PAGE_SIZE_BYTES) {
    Serial.println("EEPROM ERROR: decrease MEMORY_BUFFER_RECORDS, write size is higher than PAGE_SIZE.");
    return SLEEP;
  }

  eeprom_write(eeprom_write_adr, buffer, MEMORY_BUFFER_RECORDS);
  eeprom_write_adr += MEMORY_PAGE_SIZE_BYTES;

  return DEPLOYED;
}

void memory_read() {
  Serial.println("Reading fall data from EEPROM:");
  Serial.print("Time scale between data: ");
  Serial.print(MPU_READ_PERIOD_US);
  Serial.println(" us");
  Serial.print("Number of records: ");
  Serial.println(MEMORY_BUFFER_RECORDS * 512);

  int data_size = sizeof(struct data_t);
  int address = 0;
  int record_index = 0;

  while (address + data_size <= MEMORY_SIZE_BYTES) {

    eeprom_read(address, buffer, MEMORY_BUFFER_RECORDS);

    for (int i = 0; i < MEMORY_BUFFER_RECORDS; i++) {
      Serial.print(record_index);
      Serial.print(",");
      Serial.print(buffer[i].x);
      Serial.print(",");
      Serial.print(buffer[i].y);
      Serial.print(",");
      Serial.println(buffer[i].z);

      record_index++;
    }

    address += MEMORY_PAGE_SIZE_BYTES;
  }
  Serial.println("End of data");
}

void serial_read() {
  while (Serial.available() > 0) {
    char inChar = (char)Serial.read();

    if (inChar == '\n' || inChar == '\r')
      is_command_complete = true;
    else
      input_command += inChar;
  }
}

void process_command() {
  if (is_command_complete) {
    // remove leading/trailing whitespace
    input_command.trim();

    if (input_command.equals("READ_DATA")) {
      memory_read();

      // Reset command variables
      input_command = "";
      is_command_complete = false;
    }
  }
}

void loop() {
  switch (state) {
    // read the magnitude from the MPU
    case MEASURE:
      mpu_read();
      state = DETECT;
      break;

    // detect free fall from measured data
    case DETECT:
      // assings MEASURE/DETONATE state
      state = detect_fall();
      break;

    case DETONATE:
#if DEBUG_MODE
      Serial.print("ACTIVATE DETONATOR");
#endif
      digitalWrite(PIN_DETONATOR, HIGH);
      digitalWrite(PIN_LED, HIGH);
      detonator_start_time = millis();
      is_detonator_active = true;
      state = DEPLOYED;
      Timer1.attachInterrupt(timerISR);
      break;

    case DEPLOYED:
      // deactivete the detonator after some time TIME_TURN_OFF_MS
      if (is_detonator_active && millis() - detonator_start_time >= TIME_TURN_OFF_MS) {
#if DEBUG_MODE
        Serial.print("DEACTIVATE DETONATOR");
#endif
        digitalWrite(PIN_DETONATOR, LOW);
        is_detonator_active = false;
      }

      // triggered by ISR
      if (is_read_flag) {
        Timer1.detachInterrupt();
        mpu_read();

        buffer[buffer_adr].x = acc_x;
        buffer[buffer_adr].y = acc_y;
        buffer[buffer_adr].z = acc_z;
        buffer_adr++;

#if DEBUG_MODE
        Serial.print(acc_x);
        Serial.print(",");
        Serial.print(acc_y);
        Serial.print(",");
        Serial.println(acc_z);
#endif

        // if RAM buffer is full store it in the eeprom
        if (buffer_adr >= MEMORY_BUFFER_RECORDS) {
          // assings DEPLOYED or SLEEP state when memory is full
          state = memory_store();
          buffer_adr = 0;
        }
        is_read_flag = false;
        Timer1.attachInterrupt(timerISR);
      }
      break;

    case SLEEP:
      serial_read();
      process_command();

      // non-blocking LED blink
      static unsigned long previous_time = 0;
      static bool led_state = LOW;

      unsigned long current_time = millis();
      if (current_time - previous_time >= BLINK_PERIOD_MS) {
        previous_time = current_time;
        led_state = !led_state;
        digitalWrite(PIN_LED, led_state);
      }
      break;
  }
}

void timerISR() {
  is_read_flag = true;
}
