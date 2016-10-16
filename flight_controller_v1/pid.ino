#include <PID_v1.h>

//Define Variables we'll be connecting to
double pidSetpoint, pidInput, pidOutput;

//Specify the links and initial tuning parameters
double Kp= 0.001, Ki= 0.001, Kd= 0.002;

PID xPID(&pidInput, &pidOutput, &pidSetpoint, Kp, Ki, Kd, DIRECT);

void pidSetup()
{
  xPID.SetMode(AUTOMATIC);
  xPID.SetOutputLimits(-90,90);
  xPID.SetSampleTime(5);
}

void pidStage(float rollAngle, float* motorSpeeds, float controlAngle)
{
  pidInput = rollAngle;
  pidSetpoint = controlAngle;
  if (xPID.Compute())
  {
    (motorSpeeds)[0] += pidOutput;
    (motorSpeeds)[1] -= pidOutput;
    (motorSpeeds)[2] += pidOutput;
    (motorSpeeds)[3] -= pidOutput;
  }
}
