#pragma once

class PIDController
{
public:
  PIDController(float period, float minControl, float maxControl);

  void update(float angle, float target);

  const float period;
  float kP, kI, kD;
  float control;

private:
  float oldError, oldOldError;
};
