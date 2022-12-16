#include "ComplementaryFilter.h"

#include <cmath>

// Complementary filter for computing angle from accelerometer and gyroscope measurements
// Algorithm is based on Pieter-Jan Van de Maele's blog article https://www.pieter-jan.com/node/11

ComplementaryFilter::ComplementaryFilter(float samplingFreq, float accelWeight)
  : deltaTime(1.0f / samplingFreq), accelWeight(accelWeight), gyroWeight(1.0f - accelWeight), angle(0.0f)
{
}

float ComplementaryFilter::update(float accelX, float accelY, float gyro)
{
  // Calculate angle using gyro measurement
  float gyroAngle = angle + gyro * deltaTime;
  // Calculate angle using accelerometer measurements
  float accelAngle = atan2(accelX, accelY);
  // Mix angle from two sources
  angle = gyroWeight * gyroAngle + accelWeight * accelAngle;

  return angle;
}