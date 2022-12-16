#include "PIDController.h"

PIDController::PIDController(float period, float minControl, float maxControl)
  : period(period)
{
}

void PIDController::update(float angle, float target)
{
  // 速度型PID制御
  float error = target - angle;
  float diffError = (error - oldError) / period;  // 誤差の微分
  float diffDiffError = (error - 2 * oldError + oldOldError) / (period * period);  // 誤差の2階微分
  float diffControl = kP * diffError + kI * error + kD * diffDiffError;
  control += diffControl * period;
  
  oldOldError = oldError;
  oldError = error;
}
