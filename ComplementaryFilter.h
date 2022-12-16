#pragma once

// Complementary filter for computing angle from accelerometer and gyroscope measurements
// Algorithm is based on Pieter-Jan Van de Maele's blog article https://www.pieter-jan.com/node/11
class ComplementaryFilter
{
public:
  // Constructor
  // samplingFreq should be specified in Hz.
  ComplementaryFilter(float samplingFreq, float accelWeight = 0.02);
  
  // Update angle using sensor measurements.
  // +X is left, +Y is up, positive angle is counterclockwise.
  // gyro is angular velocity in rad/s.
  // Returns angle in radians.
  float update(float accelX, float accelY, float gyro);

  float angle;

private:
  const float deltaTime;
  const float accelWeight;
  const float gyroWeight;
};