/*
 RESCUE ROBOT - PROTOTYPE

 PIN MAP
 D2  = HC-SR04 shared TRIG/ECHO Newping
 D3  = L298N LEFT IN1
 D4  = QTR Sensor 7
 D5  = L298N LEFT IN2
 D7  = QTR Sensor 5
 D8  = Servo
 D10 = SD CS
 D11 = SD MOSI
 D12 = SD MISO
 D13 = SD SCK

 Analog Pins
 A0  = QTR Sensor 3
 A1  = QTR Sensor 1
 A2  = HX711 DOUT
 A3  = HX711 SCK
 A4  = I2C SDA
 A5  = I2C SCL

 I2C DEVICES
 LCD     = 0x27
 EEPROM  = 0x50
 DS3231  = 0x68
 MPU6050 = 0x69

 Serial Monitor = 115200 baud
*/

// Options
// Compile-time switches remove entire subsystems from the program when set to 0.
#define ENABLE_LCD 1
#define ENABLE_SD 1

// Calibration Mode
// Set to 1 only when calculating a new HX711 calibration factor.
#define CALIBRATE_LOAD_CELL 0

// Libraries
#include <Wire.h>
#include <Servo.h>
#include <HX711.h>

#if ENABLE_SD
#include <SPI.h>
#include <SD.h>
#endif

// Pin definitions
#define SONAR_PIN 2
#define LEFT_IN1 3
#define LEFT_IN2 5
#define SERVO_PIN 8
#define SD_CS 10
#define HX_DOUT A2
#define HX_SCK A3

// I2C addresses
#define LCD_ADDR 0x27
#define RTC_ADDR 0x68
#define MPU_ADDR 0x69

// Motor configuration
const int MOTOR_SPEED = 150;

// Servo configuration
const int BLADE_UP_ANGLE = 30;
const int BLADE_DOWN_ANGLE = 100;

// QTR configuration
const byte QTR_COUNT = 4;
const unsigned int QTR_TIMEOUT_US = 4500;

// Load cell configuration
const float KNOWN_MASS_G = 50.0;
const float LOAD_CELL_FACTOR = -673.42401;

// MPU6050 configuration
const float HEADING_DIRECTION = 1.0;

#if ENABLE_LCD
#define LCD_RS 0x01
#define LCD_EN 0x04
#define LCD_BACKLIGHT 0x08
#endif

// The enum stores motor direction as named states instead of unexplained numeric values.
enum MotorState {
  MOTOR_STOPPED,
  MOTOR_FORWARD,
  MOTOR_REVERSE
};

// The F() macro keeps constant text in flash memory instead of consuming scarce SRAM.
void divider(){
  Serial.println(F("============================================================"));
}

void pass(const __FlashStringHelper *message){
  Serial.print(F("[PASS] "));
  Serial.println(message);
}

void fail(const __FlashStringHelper *message){
  Serial.print(F("[FAIL] "));
  Serial.println(message);
}

// A zero I2C endTransmission result means a device acknowledged the requested address.
bool i2cPresent(byte address){
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

// The scanner probes every valid 7-bit I2C address to verify devices on the shared bus.
void scanI2C(){
  Serial.println();
  divider();
  Serial.println(F("I2C SYSTEM SCAN"));
  divider();
  byte count = 0;

  for (byte address = 1; address < 127; address++){
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0){
      Serial.print(F("[FOUND] 0x"));
      if (address < 16) Serial.print('0');
      Serial.print(address, HEX);
      if (address == 0x27) Serial.print(F(" = LCD"));
      else if (address == 0x50) Serial.print(F(" = RTC EEPROM"));
      else if (address == 0x68) Serial.print(F(" = DS3231"));
      else if (address == 0x69) Serial.print(F(" = MPU6050"));
      Serial.println();
      count++;
    }
  }

  Serial.print(F("[RESULT] Devices found: "));
  Serial.println(count);
}

// This class owns QTR calibration, RC timing, normalisation and weighted line-position processing.
class QTRArray {
private:
  byte pins[QTR_COUNT];
  unsigned int raw[QTR_COUNT] = {0, 0, 0, 0};
  unsigned int values[QTR_COUNT] = {0, 0, 0, 0};
  unsigned int minimums[QTR_COUNT] = {0, 0, 0, 0};
  unsigned int maximums[QTR_COUNT] = {0, 0, 0, 0};
  unsigned int position = 1500;

  // Each RC sensor capacitor is charged HIGH before timing how quickly reflected IR discharges it.
  void readRaw(){
    for (byte i = 0; i < QTR_COUNT; i++){
      pinMode(pins[i], OUTPUT);
      digitalWrite(pins[i], HIGH);
    }
    delayMicroseconds(10);

    // micros() provides microsecond timing required for the QTR RC discharge measurement.
    unsigned long start = micros();
    bool finished[QTR_COUNT] = {false, false, false, false};
    byte remaining = QTR_COUNT;

    // Switching to INPUT releases the charged line so the capacitor can discharge through the sensor.
    for (byte i = 0; i < QTR_COUNT; i++){
      pinMode(pins[i], INPUT);
      digitalWrite(pins[i], LOW);
    }

    // The loop records the exact discharge time of each sensor until all finish or the timeout is reached.
    while (remaining > 0 && micros() - start < QTR_TIMEOUT_US){
      unsigned int elapsed = micros() - start;
      for (byte i = 0; i < QTR_COUNT; i++){
        if (!finished[i] && digitalRead(pins[i]) == LOW){
          raw[i] = elapsed;
          finished[i] = true;
          remaining--;
        }
      }
    }

    // A sensor that never falls LOW is assigned the timeout value to represent maximum darkness.
    for (byte i = 0; i < QTR_COUNT; i++){
      if (!finished[i]) raw[i] = QTR_TIMEOUT_US;
    }
  }

public:
  QTRArray(byte sensor7, byte sensor5, byte sensor3, byte sensor1){
    pins[0] = sensor7;
    pins[1] = sensor5;
    pins[2] = sensor3;
    pins[3] = sensor1;
  }

  // Calibration records each sensor's observed black/white limits to compensate for sensor variation.
  void calibrate(){
    Serial.println();
    divider();
    Serial.println(F("QTR REFLECTANCE CALIBRATION"));
    divider();
    Serial.println(F("Move sensors across BLACK and WHITE."));
    Serial.println(F("Calibration starts in 2 seconds."));
    delay(2000);

    for (byte i = 0; i < QTR_COUNT; i++){
      minimums[i] = QTR_TIMEOUT_US;
      maximums[i] = 0;
    }

    for (int sample = 0; sample < 300; sample++){
      readRaw();
      for (byte i = 0; i < QTR_COUNT; i++){
        if (raw[i] < minimums[i]) minimums[i] = raw[i];
        if (raw[i] > maximums[i]) maximums[i] = raw[i];
      }
      delay(10);
    }

    pass(F("QTR calibration complete."));
    Serial.println(F("Sensor order = 7, 5, 3, 1"));
    Serial.print(F("MIN: "));
    for (byte i = 0; i < QTR_COUNT; i++){ Serial.print(minimums[i]); Serial.print(' '); }
    Serial.println();
    Serial.print(F("MAX: "));
    for (byte i = 0; i < QTR_COUNT; i++){ Serial.print(maximums[i]); Serial.print(' '); }
    Serial.println();
  }

  void update(){
    readRaw();
    // A weighted centroid combines four calibrated sensors into one continuous line-position value.
    unsigned long weightedTotal = 0;
    unsigned long sensorTotal = 0;

    for (byte i = 0; i < QTR_COUNT; i++){
      long range = maximums[i] - minimums[i];
      if (range < 10){
        values[i] = 0;
        continue;
      }

      // Raw discharge time is normalised to a common 0-1000 reflectance scale.
      long value = ((long)raw[i] - minimums[i]) * 1000L / range;
      value = constrain(value, 0, 1000);
      values[i] = value;
      sensorTotal += value;
      weightedTotal += (unsigned long)value * i * 1000UL;
    }

    // Weak total reflectance is treated as insufficient evidence to update the previous line position.
    if (sensorTotal < 200) return;
    position = weightedTotal / sensorTotal;
  }

  unsigned int getValue(byte index) const { return values[index]; }
  unsigned int getPosition() const { return position; }

  unsigned long signalTotal() const {
    unsigned long sum = 0;
    for (byte i = 0; i < QTR_COUNT; i++) sum += values[i];
    return sum;
  }

  // Returning flash-string pointers avoids duplicating line-state text in Serial output code.
  const __FlashStringHelper* stateText() const {
    if (signalTotal() < 200) return F("LOST");
    if (position < 500) return F("FAR_LEFT");
    if (position < 1100) return F("LEFT");
    if (position <= 1900) return F("CENTER");
    if (position <= 2500) return F("RIGHT");
    return F("FAR_RIGHT");
  }

  void printState(Print &out) const { out.print(stateText()); }
};

// This class owns shared-pin trigger/echo timing, distance conversion and object-zone classification.
class UltrasonicSensor {
private:
  byte pin;
  unsigned int distance = 0;

public:
  UltrasonicSensor(byte sharedPin) : pin(sharedPin) {}

  void update(){
    // The shared pin changes from OUTPUT for the trigger pulse to INPUT for the returning echo.
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
    delayMicroseconds(2);
    digitalWrite(pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(pin, LOW);
    pinMode(pin, INPUT);

    // pulseIn() measures round-trip echo width and returns zero when the 15 ms timeout expires.
    unsigned long duration = pulseIn(pin, HIGH, 15000UL);
    if (duration == 0){
      distance = 0;
      return;
    }

    // Echo time is multiplied by sound speed and divided by two for the outward and return paths.
    distance = (unsigned int)(duration * 0.0343 / 2.0);
  }

  unsigned int getDistance() const { return distance; }

  // Distance thresholds convert a numerical measurement into operational object-detection zones.
  const __FlashStringHelper* stateText() const {
    if (distance == 0) return F("NO_ECHO");
    if (distance <= 5) return F("CONTACT_ZONE");
    if (distance <= 25) return F("NEAR");
    if (distance <= 50) return F("OBJECT");
    return F("CLEAR");
  }

  void printState(Print &out) const { out.print(stateText()); }
};

// This class encapsulates HX711 communication, calibration, mass conversion and force calculation.
class LoadCellSensor {
private:
  HX711 scale;
  bool working = false;
  float activeFactor = 1.0;
  float weightGrams = 0;
  float forceNewtons = 0;

public:
  void begin(){
    Serial.println();
    divider();
    Serial.println(F("HX711 LOAD CELL"));
    divider();
    scale.begin(HX_DOUT, HX_SCK);
    Serial.println(F("[HX711] Waiting for sensor..."));

    // wait_ready_timeout() avoids treating the HX711 as failed before its first conversion becomes ready.
    if (!scale.wait_ready_timeout(2000)){
      fail(F("HX711 did not respond within 2 seconds."));
      working = false;
      return;
    }

    working = true;
    pass(F("HX711 detected and ready."));
    working = true;
    pass(F("HX711 detected."));
    Serial.println(F("Remove all force/weight."));
    delay(3000);

    // Taring averages unloaded readings so later measurements are relative to a zero-force baseline.
    scale.set_scale(1.0);
    scale.tare(15);
    pass(F("Load cell tare complete."));

#if CALIBRATE_LOAD_CELL
    Serial.println();
    Serial.print(F("Place exactly "));
    Serial.print(KNOWN_MASS_G, 1);
    Serial.println(F(" g on load cell."));
    Serial.println(F("Reading begins in 6 seconds..."));
    delay(6000);

    // Dividing the averaged ADC change by the known mass produces counts-per-gram calibration.
    float rawDifference = scale.get_value(15);
    activeFactor = rawDifference / KNOWN_MASS_G;
    if (activeFactor > -0.01 && activeFactor < 0.01){
      fail(F("Invalid load-cell calibration."));
      working = false;
      return;
    }

    scale.set_scale(activeFactor);
    Serial.println();
    pass(F("Load cell calibrated."));
    Serial.print(F("CALIBRATION FACTOR = "));
    Serial.println(activeFactor, 5);
    Serial.println();
    Serial.println(F("COPY THAT NUMBER INTO LOAD_CELL_FACTOR"));
    Serial.println(F("THEN CHANGE CALIBRATE_LOAD_CELL TO 0."));
#else
    activeFactor = LOAD_CELL_FACTOR;
    scale.set_scale(activeFactor);
    Serial.print(F("Using saved calibration factor: "));
    Serial.println(activeFactor, 5);
#endif
  }

  void update(){
    // Missing one fresh HX711 conversion skips only that cycle instead of marking the sensor offline.
    if (!working || !scale.is_ready()) return;
    float newWeight = scale.get_units(1);

    // A +/-2 g deadband suppresses small zero-point noise around the unloaded state.
    if (newWeight > -2.0 && newWeight < 2.0) newWeight = 0.0;
    weightGrams = newWeight;

    // Mass is converted from grams to kilograms before applying F = m × g.
    forceNewtons = (weightGrams / 1000.0) * 9.80665;
  }

  bool isWorking() const { return working; }
  float getWeight() const { return weightGrams; }
  float getForce() const { return forceNewtons; }
  float getCalibrationFactor() const { return activeFactor; }
};

// This class handles direct MPU6050 register access, gyro bias correction and relative yaw integration.
class MPU6050Sensor {
private:
  byte address;
  bool working = false;
  float gyroZBias = 0;
  float headingDegrees = 0;
  float temperature = 0;
  unsigned long previousMicros = 0;

  // Direct register writes replace the larger MPU6050 library and reduce Arduino flash usage.
  // Finds part,then register,then assigns value
  bool writeRegister(byte reg, byte value){
    Wire.beginTransmission(address);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
  }

  bool readRaw(float &gyroZDegrees, float &temperatureC){
    // Register 0x41 starts a sequential read containing temperature followed by X, Y and Z gyro data.
    Wire.beginTransmission(address);
    Wire.write(0x41);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(address, (byte)8);
    if (Wire.available() < 8) return false;

    // MPU values are signed 16-bit numbers stored as separate high and low bytes.
    int16_t rawTemp = ((int16_t)Wire.read() << 8) | Wire.read();

    // X and Y gyro bytes are discarded because only Z-axis yaw is required.
    //6 bytes total; it discards the first 2 X and the 2 Y bytes.
    Wire.read(); Wire.read();
    Wire.read(); Wire.read();
   //combines the 2 8-bit Z bytes into a 16-bit signed number by shifting the high byte by 8.
    int16_t rawGyroZ = ((int16_t)Wire.read() << 8) | Wire.read();

    // At the configured +/-500 deg/s range the gyro sensitivity is 65.5 LSB per deg/s.
    gyroZDegrees = rawGyroZ / 65.5;

    // The temperature formula is defined by the MPU6050 register specification.
    temperatureC = rawTemp / 340.0 + 36.53;
    return true;
  }

public:
  MPU6050Sensor(byte i2cAddress) : address(i2cAddress) {}

  void begin(){
    Serial.println();
    divider();
    Serial.println(F("MPU6050 GYROSCOPE"));
    divider();

    if (!i2cPresent(address)){
      fail(F("MPU6050 0x69 not detected."));
      working = false;
      return;
    }

    // These registers from the datasheet wake the MPU, enable digital filtering and select the +/-500 deg/s gyro range.
    //0x6B = PWR_MGMT_1(Bit 6), 0x00 = 00000000
    writeRegister(0x6B, 0x00);// wakes up mpu
    //0x1A = DLPF_CFG(bottom 3 bits), 0x03 = 00000011
    writeRegister(0x1A, 0x03); //digital low pass filter set to setting 3.
    //0x1B = FS_SEL(bit4-3), 0x08 = 00001000;
    writeRegister(0x1B, 0x08); //gyroscope range set to setting 1, +_500 degrees/s
    delay(100);
    working = true;
    pass(F("MPU6050 detected."));
    Serial.println(F("Keep robot COMPLETELY STILL."));
    Serial.println(F("Calculating gyro bias..."));

    // Averaging stationary samples estimates constant Z-axis gyro bias before heading integration begins.
    float sum = 0;
    int validSamples = 0;
    for (int i = 0; i < 200; i++){
      float gyro, temp;
      if (readRaw(gyro, temp)){
        sum += gyro;
        validSamples++;
      }
      delay(5);
    }

    if (validSamples == 0){
      fail(F("MPU calibration failed."));
      working = false;
      return;
    }

    gyroZBias = sum / validSamples;
    Serial.print(F("Gyro Z bias = "));
    Serial.print(gyroZBias, 4);
    Serial.println(F(" deg/s"));
    headingDegrees = 0;
    previousMicros = micros();
    pass(F("Relative heading zeroed. Drift reduced"));
  }

  void update(){
    // Heading is produced by integrating bias-corrected angular velocity over elapsed time.
    if (!working) return;
    float gyroZ, temp;
    if (!readRaw(gyroZ, temp)) return;

    unsigned long now = micros();

    // Microseconds are converted to seconds so deg/s multiplied by dt produces degrees.
    float dt = (now - previousMicros) / 1000000.0;
    previousMicros = now;
    gyroZ -= gyroZBias;

    // A small angular-velocity deadband reduces heading drift caused by stationary gyro noise.
    if (gyroZ > -0.5 && gyroZ < 0.5) gyroZ = 0;
    //angle = angular velocity * time;
    headingDegrees += gyroZ * dt * HEADING_DIRECTION;

    // Wrapping prevents heading from growing outside the conventional 0-359 degree range.
    while (headingDegrees >= 360) headingDegrees -= 360;
    while (headingDegrees < 0) headingDegrees += 360;
    temperature = temp;
  }

  void resetTiming(){ previousMicros = micros(); }
  bool isWorking() const { return working; }
  float getHeading() const { return headingDegrees; }
  float getTemperature() const { return temperature; }

  // Direction sectors translate continuous relative yaw into human-readable orientation states.
  const __FlashStringHelper* directionText() const {
    if (headingDegrees >= 337.5 || headingDegrees < 22.5) return F("FORWARD");
    if (headingDegrees < 67.5) return F("FRONT_RIGHT");
    if (headingDegrees < 112.5) return F("RIGHT");
    if (headingDegrees < 157.5) return F("BACK_RIGHT");
    if (headingDegrees < 202.5) return F("BACK");
    if (headingDegrees < 247.5) return F("BACK_LEFT");
    if (headingDegrees < 292.5) return F("LEFT");
    return F("FRONT_LEFT");
  }

  void printDirection(Print &out) const { out.print(directionText()); }
};

// This class encapsulates DS3231 BCD conversion, clock reads and elapsed-time calculations.
class RTCClock {
private:
  byte address;
  bool working = false;
  unsigned long startSeconds = 0;

  // DS3231 time registers use Binary-Coded Decimal, so each nibble stores one decimal digit.
  // top 4 bits is upper nibble, bottom 4 bits is lower nibble.
  byte bcdToDecimal(byte value) const { return (value >> 4) * 10 + (value & 0x0F); }

public:
  RTCClock(byte i2cAddress) : address(i2cAddress) {}

  // Passing time variables by reference lets one function return hour, minute and second together.
  bool readTime(byte &hour, byte &minute, byte &second){
    Wire.beginTransmission(address);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(address, (byte)3);
    if (Wire.available() < 3) return false;

    second = bcdToDecimal(Wire.read() & 0x7F);
    minute = bcdToDecimal(Wire.read() & 0x7F);
    byte rawHour = Wire.read();

    // Bit 6 selects 12-hour mode, requiring separate handling of the AM/PM flag.
    if (rawHour & 0x40){
      bool pm = rawHour & 0x20;
      hour = bcdToDecimal(rawHour & 0x1F);
      if (pm && hour != 12) hour += 12;
      if (!pm && hour == 12) hour = 0;
    } else hour = bcdToDecimal(rawHour & 0x3F);

    return true;
  }

  void printTwoDigits(Print &out, byte value) const {
    if (value < 10) out.print('0');
    out.print(value);
  }

  void begin(){
    Serial.println();
    divider();
    Serial.println(F("DS3231 RTC"));
    divider();

    if (!i2cPresent(address)){
      fail(F("DS3231 not detected."));
      working = false;
      return;
    }

    byte h, m, s;
    if (!readTime(h, m, s)){
      fail(F("RTC time read failed."));
      working = false;
      return;
    }

    working = true;
    startSeconds = (unsigned long)h * 3600UL + (unsigned long)m * 60UL + s;
    pass(F("DS3231 detected."));
    Serial.print(F("Start time = "));
    printTwoDigits(Serial, h); Serial.print(':');
    printTwoDigits(Serial, m); Serial.print(':');
    printTwoDigits(Serial, s); Serial.println();
  }

  // RTC time is converted to seconds since midnight so subtraction produces elapsed test duration.
  unsigned long elapsedSeconds(){
    if (!working) return millis() / 1000UL;
    byte h, m, s;
    if (!readTime(h, m, s)) return millis() / 1000UL;
    unsigned long nowSeconds = (unsigned long)h * 3600UL + (unsigned long)m * 60UL + s;

    // Adding 86400 seconds handles a test that crosses midnight without producing a negative duration.
    if (nowSeconds >= startSeconds) return nowSeconds - startSeconds;
    return 86400UL - startSeconds + nowSeconds;
  }

  void printElapsed(Print &out, unsigned long total) const {
    unsigned long hours = total / 3600UL;
    byte minutes = (total % 3600UL) / 60UL;
    byte seconds = total % 60UL;
    if (hours < 10) out.print('0');
    out.print(hours); out.print(':');
    printTwoDigits(out, minutes); out.print(':');
    printTwoDigits(out, seconds);
  }

  bool isWorking() const { return working; }
};

// This class owns L298N direction control and the non-blocking motor demonstration state machine.
class MotorController {
private:
  byte in1;
  byte in2;
  int speedSetting;
  MotorState state = MOTOR_STOPPED;
  byte phase = 0;
  unsigned long phaseStart = 0;

  // Positive PWM drives forward, negative PWM reverses H-bridge polarity and zero stops the motor.
  void setSpeed(int speedValue){
    speedValue = constrain(speedValue, -255, 255);
    if (speedValue > 0){
      analogWrite(in1, speedValue);
      digitalWrite(in2, LOW);
      state = MOTOR_FORWARD;
    } else if (speedValue < 0){
      digitalWrite(in1, LOW);
      analogWrite(in2, -speedValue);
      state = MOTOR_REVERSE;
    } else {
      digitalWrite(in1, LOW);
      digitalWrite(in2, LOW);
      state = MOTOR_STOPPED;
    }
  }

  // One helper updates the motor state machine without repeating the same transition code four times.
  void nextPhase(byte nextPhase, int speed, const __FlashStringHelper *message, unsigned long now){
    phase = nextPhase;
    phaseStart = now;
    setSpeed(speed);
    Serial.println(message);
  }

public:
  MotorController(byte input1, byte input2, int pwmSpeed) : in1(input1), in2(input2), speedSetting(pwmSpeed) {}

  void begin(){
    pinMode(in1, OUTPUT);
    pinMode(in2, OUTPUT);
    setSpeed(0);
  }

  void resetTiming(unsigned long now){ phaseStart = now; }

  void update(){
    // The phase state machine sequences stop, forward, stop and reverse without using blocking delays(finite-state machine).
    unsigned long now = millis();
    unsigned long elapsed = now - phaseStart;
    if (phase == 0 && elapsed >= 3000) nextPhase(1, speedSetting, F("[MOTOR] LEFT -> FORWARD"), now);
    else if (phase == 1 && elapsed >= 2000) nextPhase(2, 0, F("[MOTOR] LEFT -> STOP"), now);
    else if (phase == 2 && elapsed >= 2000) nextPhase(3, -speedSetting, F("[MOTOR] LEFT -> REVERSE"), now);
    else if (phase == 3 && elapsed >= 2000) nextPhase(0, 0, F("[MOTOR] LEFT -> STOP"), now);
  }

  MotorState getState() const { return state; }
  int getPWM() const { return state == MOTOR_STOPPED ? 0 : speedSetting; }
};

// This class encapsulates servo position, blade state and its non-blocking timed demonstration.
class BladeServo {
private:
  Servo servo;
  byte pin;
  bool bladeDown = false;
  int angle = BLADE_UP_ANGLE;
  unsigned long lastChange = 0;

public:
  BladeServo(byte servoPin) : pin(servoPin) {}

  void begin(){
    servo.attach(pin);
    angle = BLADE_UP_ANGLE;
    servo.write(angle);
  }

  void resetTiming(unsigned long now){ lastChange = now; }

  void update(){
    // The servo toggles between stored blade angles every eight seconds using non-blocking timing.
    unsigned long now = millis();
    if (now - lastChange < 8000) return;
    lastChange = now;

    // The ! operator toggles the boolean and the ternary operator selects the matching servo angle.
    bladeDown = !bladeDown;
    angle = bladeDown ? BLADE_DOWN_ANGLE : BLADE_UP_ANGLE;
    servo.write(angle);
    Serial.print(F("[SERVO] BLADE -> "));
    Serial.println(bladeDown ? F("DOWN") : F("UP"));
  }

  bool isDown() const { return bladeDown; }
  int getAngle() const { return angle; }
};

// This timer provides reusable non-blocking scheduling for periodic sensor and output tasks.
class TaskTimer {
private:
  unsigned long lastRun = 0;

public:
  void reset(unsigned long now){ lastRun = now; }

  bool due(unsigned long interval, unsigned long now){
    // Unsigned subtraction remains reliable even when the millis() counter eventually overflows.
    if (now - lastRun < interval) return false;
    lastRun = now;
    return true;
  }
};

#if ENABLE_LCD
// This compact class encapsulates low-level PCF8574 LCD communication and rotating live-data pages.
class CompactLCD {
private:
  byte address;
  bool working = false;
  byte page = 0;

  void expander(byte data){
    Wire.beginTransmission(address);
    Wire.write(data | LCD_BACKLIGHT);
    Wire.endTransmission();
  }

  // The enable pulse tells the HD44780 controller when to latch each 4-bit data nibble.
  void pulse(byte data){
    expander(data | LCD_EN);
    delayMicroseconds(1);
    expander(data & ~LCD_EN);
    delayMicroseconds(50);
  }

  void write4(byte data){ expander(data); pulse(data); }

  // Each 8-bit LCD command or character is transmitted as two 4-bit nibbles.
  void send(byte value, byte mode){
    write4((value & 0xF0) | mode);
    write4(((value << 4) & 0xF0) | mode);
  }

  void command(byte value){ send(value, 0); }
  void character(char c){ send(c, LCD_RS); }

  void text(const char *value){
    while (*value) character(*value++);
  }

  void number(long value){
    char buffer[12];
    ltoa(value, buffer, 10);
    text(buffer);
  }

  // HD44780 row addresses differ in DDRAM, so row 2 begins at offset 0x40.
  void cursor(byte column, byte row){ command(0x80 | (column + (row == 0 ? 0x00 : 0x40))); }
  void clear(){ command(0x01); delay(2); }

  void lineState(const QTRArray &qtr){
    if (qtr.signalTotal() < 200) text("LOST");
    else if (qtr.getPosition() < 500) text("FAR LEFT");
    else if (qtr.getPosition() < 1100) text("LEFT");
    else if (qtr.getPosition() <= 1900) text("CENTER");
    else if (qtr.getPosition() <= 2500) text("RIGHT");
    else text("FAR RIGHT");
  }

  void headingState(const MPU6050Sensor &mpu){
    float heading = mpu.getHeading();
    if (heading >= 337.5 || heading < 22.5) text("FORWARD");
    else if (heading < 112.5) text("RIGHT");
    else if (heading < 247.5) text("BACK");
    else text("LEFT");
  }

public:
  CompactLCD(byte i2cAddress) : address(i2cAddress) {}

  void begin(){
    if (!i2cPresent(address)){
      working = false;
      fail(F("LCD 0x27 not detected."));
      return;
    }

    working = true;

    // The startup sequence forces the LCD from its default 8-bit state into 4-bit communication mode.
    delay(50);
    write4(0x30); delayMicroseconds(4500);
    write4(0x30); delayMicroseconds(4500);
    write4(0x30); delayMicroseconds(150);
    write4(0x20);
    command(0x28);
    command(0x08);
    clear();
    command(0x06);
    command(0x0C);
    cursor(0, 0); text("FINAL PROTOTYPE");
    cursor(0, 1); text("INITIALISING");
    pass(F("LCD detected."));
  }

  // page rotates subsystem summaries because a 16x2 display cannot show all data at once.
  void update(RTCClock &rtc, const UltrasonicSensor &sonar, const QTRArray &qtr, const LoadCellSensor &loadCell, const MPU6050Sensor &mpu, const MotorController &motor, const BladeServo &blade){
    if (!working) return;
    clear();

    if (page == 0){
      unsigned long elapsed = rtc.elapsedSeconds();
      unsigned int minutes = (elapsed / 60UL) % 100;
      byte seconds = elapsed % 60;
      cursor(0, 0); text("TIME ");
      if (minutes < 10) character('0');
      number(minutes); character(':');
      if (seconds < 10) character('0');
      number(seconds);
      cursor(0, 1); text("DIST ");
      if (sonar.getDistance() == 0) text("NONE");
      else { number(sonar.getDistance()); text("cm"); }
    } else if (page == 1){
      cursor(0, 0); text("WEIGHT ");
      if (loadCell.isWorking()){ number((long)loadCell.getWeight()); text("g"); }
      else text("FAIL");
      cursor(0, 1); text("FORCE ");
      if (loadCell.isWorking()){
        long milliNewtons = loadCell.getForce() * 1000.0;
        number(milliNewtons); text("mN");
      }
    } else if (page == 2){
      cursor(0, 0); text("LINE "); number(qtr.getPosition());
      cursor(0, 1); lineState(qtr);
    } else if (page == 3){
      cursor(0, 0); text("HEADING ");
      if (mpu.isWorking()){ number((long)mpu.getHeading()); character((char)223); }
      else text("FAIL");
      cursor(0, 1);
      if (mpu.isWorking()) headingState(mpu);
    } else {
      cursor(0, 0); text("MOTOR ");
      if (motor.getState() == MOTOR_FORWARD) text("FWD");
      else if (motor.getState() == MOTOR_REVERSE) text("REV");
      else text("STOP");
      cursor(0, 1); text("BLADE "); text(blade.isDown() ? "DOWN" : "UP");
    }

    page++;
    if (page > 4) page = 0;
  }

  bool isWorking() const { return working; }
};
#endif

#if ENABLE_SD
// This class owns SD initialisation, mission-file reading and persistent CSV logging.
class SDLogger {
private:
  bool working = false;

public:
  void begin(){
    Serial.println();
    divider();
    Serial.println(F("SD CARD"));
    divider();

    // SD.begin() initialises the SPI card interface before mission reading and CSV data logging.
    if (!SD.begin(SD_CS)){
      working = false;
      fail(F("SD card failed."));
      return;
    }

    working = true;
    pass(F("SD card detected."));

    // MISSION.TXT demonstrates removable instruction transfer without recompiling the Arduino program.
    if (SD.exists("MISSION.TXT")){
      File mission = SD.open("MISSION.TXT");
      if (mission){
        Serial.println(F("----- MISSION.TXT -----"));
        while (mission.available()) Serial.write(mission.read());
        Serial.println();
        Serial.println(F("-----------------------"));
        mission.close();
      }
    } else Serial.println(F("[SD] MISSION.TXT not found."));

    // PROTO.CSV stores sensor and actuator values in a spreadsheet-compatible comma-separated format.
    File file = SD.open("PROTO.CSV", FILE_WRITE);
    if (file){
      if (file.size() == 0) file.println(F("elapsed_s,distance_cm,qtr7,qtr5,qtr3,qtr1,line_position,weight_g,force_N,heading_deg,temperature_C,motor_state,servo_angle"));
      file.close();
      pass(F("PROTO.CSV ready."));
    }
  }

  // One timestamped CSV row is appended each logging cycle so test data remains persistent after shutdown.
  void log(RTCClock &rtc, const UltrasonicSensor &sonar, const QTRArray &qtr, const LoadCellSensor &loadCell, const MPU6050Sensor &mpu, const MotorController &motor, const BladeServo &blade){
    if (!working) return;
    File file = SD.open("PROTO.CSV", FILE_WRITE);
    if (!file) return;

    file.print(rtc.elapsedSeconds()); file.print(',');
    file.print(sonar.getDistance()); file.print(',');
    for (byte i = 0; i < QTR_COUNT; i++){ file.print(qtr.getValue(i)); file.print(','); }
    file.print(qtr.getPosition()); file.print(',');
    if (loadCell.isWorking()) file.print(loadCell.getWeight(), 1);
    file.print(',');
    if (loadCell.isWorking()) file.print(loadCell.getForce(), 3);
    file.print(',');
    if (mpu.isWorking()) file.print(mpu.getHeading(), 1);
    file.print(',');
    if (mpu.isWorking()) file.print(mpu.getTemperature(), 1);
    file.print(',');
    file.print((int)motor.getState()); file.print(',');
    file.println(blade.getAngle());
    file.close();
  }

  bool isWorking() const { return working; }
};
#endif

// Arduino Serial Monitor has no true clear command, so blank lines push previous dashboard data off-screen.
class SerialDashboard {
private:
  void clear(){
    for (byte i = 0; i < 30; i++) Serial.println();
  }

public:
  void print(RTCClock &rtc, const QTRArray &qtr, const UltrasonicSensor &sonar, const LoadCellSensor &loadCell, const MPU6050Sensor &mpu, const MotorController &motor, const BladeServo &blade
#if ENABLE_SD
  , const SDLogger &sd
#endif
#if ENABLE_LCD
  , const CompactLCD &lcd
#endif
  ){
    clear();
    Serial.println();
    divider();
    Serial.println(F("FINAL PROTOTYPE - LIVE DATA"));
    divider();

    Serial.print(F("[RTC] Elapsed: "));
    rtc.printElapsed(Serial, rtc.elapsedSeconds());
    if (rtc.isWorking()){
      byte h, m, s;
      if (rtc.readTime(h, m, s)){
        Serial.print(F(" | Clock: "));
        rtc.printTwoDigits(Serial, h); Serial.print(':');
        rtc.printTwoDigits(Serial, m); Serial.print(':');
        rtc.printTwoDigits(Serial, s);
      }
    }
    Serial.println();

    Serial.print(F("[QTR] 7:")); Serial.print(qtr.getValue(0));
    Serial.print(F(" 5:")); Serial.print(qtr.getValue(1));
    Serial.print(F(" 3:")); Serial.print(qtr.getValue(2));
    Serial.print(F(" 1:")); Serial.print(qtr.getValue(3));
    Serial.print(F(" | Position:")); Serial.print(qtr.getPosition());
    Serial.print(F("/3000 | ")); qtr.printState(Serial); Serial.println();

    Serial.print(F("[ULTRASONIC] "));
    if (sonar.getDistance() == 0) Serial.print(F("NO ECHO"));
    else { Serial.print(sonar.getDistance()); Serial.print(F(" cm")); }
    Serial.print(F(" | ")); sonar.printState(Serial); Serial.println();

    Serial.print(F("[LOAD CELL] "));
    if (loadCell.isWorking()){
      Serial.print(loadCell.getWeight(), 1); Serial.print(F(" g | "));
      Serial.print(loadCell.getForce(), 3); Serial.println(F(" N"));
    } else Serial.println(F("OFFLINE"));

    Serial.print(F("[MPU6050] Heading:"));
    if (mpu.isWorking()){
      Serial.print(mpu.getHeading(), 1); Serial.print(F(" deg | Direction:"));
      mpu.printDirection(Serial); Serial.print(F(" | Temp:"));
      Serial.print(mpu.getTemperature(), 1); Serial.println(F(" C"));
    } else Serial.println(F(" OFFLINE"));

    Serial.print(F("[LEFT MOTOR] "));
    if (motor.getState() == MOTOR_FORWARD) Serial.print(F("FORWARD"));
    else if (motor.getState() == MOTOR_REVERSE) Serial.print(F("REVERSE"));
    else Serial.print(F("STOPPED"));
    Serial.print(F(" | PWM:"));
    Serial.println(motor.getPWM());

    Serial.print(F("[SERVO] ")); Serial.print(blade.getAngle());
    Serial.print(F(" deg | Blade:")); Serial.println(blade.isDown() ? F("DOWN") : F("UP"));

    Serial.print(F("[SYSTEM] RTC:")); Serial.print(rtc.isWorking() ? F("OK") : F("FAIL"));
    Serial.print(F(" MPU:")); Serial.print(mpu.isWorking() ? F("OK") : F("FAIL"));
    Serial.print(F(" HX711:")); Serial.print(loadCell.isWorking() ? F("OK") : F("FAIL"));
#if ENABLE_SD
    Serial.print(F(" SD:")); Serial.print(sd.isWorking() ? F("OK") : F("FAIL"));
#endif
#if ENABLE_LCD
    Serial.print(F(" LCD:")); Serial.print(lcd.isWorking() ? F("OK") : F("FAIL"));
#endif
    Serial.println();
  }
};

// Each object owns the state and behaviour of one hardware subsystem.
QTRArray qtr(4, 7, A0, A1);
UltrasonicSensor ultrasonic(SONAR_PIN);
LoadCellSensor loadCell;
MPU6050Sensor mpu(MPU_ADDR);
RTCClock rtc(RTC_ADDR);
MotorController motor(LEFT_IN1, LEFT_IN2, MOTOR_SPEED);
BladeServo blade(SERVO_PIN);
SerialDashboard dashboard;

#if ENABLE_LCD
CompactLCD lcd(LCD_ADDR);
#endif

#if ENABLE_SD
SDLogger sdLogger;
#endif

// Each timer stores the last execution time for one periodic task.
TaskTimer mpuTimer;
TaskTimer qtrTimer;
TaskTimer loadCellTimer;
TaskTimer serialTimer;
TaskTimer sdTimer;
TaskTimer lcdTimer;

// setup() performs one-time hardware initialisation, calibration and subsystem validation.
void setup(){
  Serial.begin(115200);
  delay(1000);
  divider();
  Serial.println(F("RESCUE ROBOT"));
  Serial.println(F("FINAL INTEGRATED PROTOTYPE"));
  Serial.println(F("MEMORY-OPTIMISED ARDUINO UNO BUILD"));
  divider();

  motor.begin();
  blade.begin();
  Wire.begin();
  scanI2C();

#if ENABLE_LCD
  lcd.begin();
#endif

  rtc.begin();
  mpu.begin();
  qtr.calibrate();
  loadCell.begin();

#if ENABLE_SD
  sdLogger.begin();
#endif

  Serial.println();
  divider();
  Serial.println(F("INITIALISATION COMPLETE"));
  Serial.println(F("INTEGRATED PROTOTYPE TEST STARTED"));
  Serial.println(F("ONLY LEFT MOTOR IS CONNECTED"));
  divider();

  unsigned long now = millis();
  motor.resetTiming(now);
  blade.resetTiming(now);
  mpuTimer.reset(now);
  qtrTimer.reset(now);
  loadCellTimer.reset(now);
  serialTimer.reset(now);
  sdTimer.reset(now);
  lcdTimer.reset(now);
  mpu.resetTiming();
}

// loop() acts as a cooperative scheduler, repeatedly servicing each subsystem when its interval expires.
void loop(){
  unsigned long now = millis();
  motor.update();
  blade.update();

  if (mpuTimer.due(20, now)) mpu.update();

  if (qtrTimer.due(200, now)){
    qtr.update();
    ultrasonic.update();
  }

  if (loadCellTimer.due(250, now)) loadCell.update();

  if (serialTimer.due(1000, now)){
    dashboard.print(rtc, qtr, ultrasonic, loadCell, mpu, motor, blade
#if ENABLE_SD
    , sdLogger
#endif
#if ENABLE_LCD
    , lcd
#endif
    );
  }

#if ENABLE_SD
  if (sdTimer.due(1000, now)) sdLogger.log(rtc, ultrasonic, qtr, loadCell, mpu, motor, blade);
#endif

#if ENABLE_LCD
  if (lcdTimer.due(2000, now)) lcd.update(rtc, ultrasonic, qtr, loadCell, mpu, motor, blade);
#endif
}
