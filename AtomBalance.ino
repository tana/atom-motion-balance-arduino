#include <Wire.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <M5Atom.h>
#include <ArduinoOSCWiFi.h>
#include "ComplementaryFilter.h"
#include "PIDController.h"

enum class MotorChannel {
  LEFT,
  RIGHT
};

const float MOTOR_MIN = -1.0f;
const float MOTOR_MAX = 1.0f;

const float NEUTRAL_ANGLE = -0.095f;
const float SPEED_TO_ANGLE = 0.01f;

const uint8_t ATOM_MOTION_ADDR = 0x38;

const int CONTROL_PERIOD_MS = 10;
const float CONTROL_FREQ = 1000.0f / float(CONTROL_PERIOD_MS);

const char WIFI_SSID[] = "AtomBalance";
const char WIFI_PASS[] = "AtomBalance";

const int OSC_PORT = 12345;

TickType_t lastTick;

ComplementaryFilter compFilter(CONTROL_FREQ);

PIDController pid(1.0f / CONTROL_FREQ, MOTOR_MIN, MOTOR_MAX);

bool motorActive = false;

float speed = 0.0f;

void setup() {
  M5.begin();
  M5.IMU.Init();

  Wire1.begin(25, 21, 400000);  // Make I2C faster

  setMotor(MotorChannel::LEFT, 0.0f);
  setMotor(MotorChannel::RIGHT, 0.0f);

  pid.kP = 10.0f;
  pid.kI = 130.0f;
  pid.kD = 0.3f;

  WiFi.softAP(WIFI_SSID, WIFI_PASS);
  Serial.print("Listening on ");
  Serial.print(WiFi.softAPIP());
  Serial.print(":");
  Serial.print(OSC_PORT);
  Serial.println();

  OscWiFi.subscribe(OSC_PORT, "/speed", speed);

  lastTick = xTaskGetTickCount();
}

void loop() {
  M5.update();
  OscWiFi.update();

  if (M5.Btn.wasPressed()) {
    motorActive = !motorActive;
  }

  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  M5.IMU.getAccelData(&accelX, &accelY, &accelZ);
  M5.IMU.getGyroData(&gyroX, &gyroY, &gyroZ);

  float angle = compFilter.update(accelZ, -accelY, gyroX * PI / 180.0f);
  
  // Stop when falled down
  if (abs(angle) > PI * 60.0f / 180.0f) {
    motorActive = false;
  }

  pid.update(angle, NEUTRAL_ANGLE + SPEED_TO_ANGLE * speed);
  
  if (motorActive) {
    pid.control = constrain(pid.control, MOTOR_MIN, MOTOR_MAX);
  } else {
    pid.control = 0.0f;
  }

  setMotor(MotorChannel::LEFT, pid.control);
  setMotor(MotorChannel::RIGHT, pid.control);

  //Serial.print(angle);
  //Serial.print('\t');
  //Serial.print(pid.control);
  //Serial.println();
  
  vTaskDelayUntil(&lastTick, pdMS_TO_TICKS(CONTROL_PERIOD_MS));
}

void setMotor(MotorChannel ch, float value) {
  if (ch == MotorChannel::LEFT) {
    value = -value;
  }

  uint8_t reg = (ch == MotorChannel::LEFT) ? 0x20 : 0x21;
  int intValue = 127 * value;

  // Wire1 is used because Atom Motion shares I2C bus with IMU
  Wire1.beginTransmission(ATOM_MOTION_ADDR);
  Wire1.write(reg);
  Wire1.write((uint8_t)constrain(intValue, -127, 127));
  Wire1.endTransmission();
}