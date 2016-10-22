#include <PID_v1.h>

//Define Variables we'll be connecting to
double pidXDesiredAngle, pidXActualAngle, pidXDesiredRate, pidXActualRate, pidXRateOutput;

//Specify the links and initial tuning parameters
double stabiliseKp= 0.01, stabiliseKi= 0.00, stabiliseKd= 0.01;
double rateKp= 0.011, rateKi= 0.00, rateKd= 0.01;

PID xStabilisePID(&pidXActualAngle, &pidXDesiredRate, &pidXDesiredAngle, stabiliseKp, stabiliseKi, stabiliseKd, DIRECT);
PID xRatePID(&pidXActualRate, &pidXRateOutput, &pidXDesiredRate, rateKp, rateKi, rateKd, DIRECT);

void tuneStabilise(double KpAdd, double KiAdd, double KdAdd)
{
  stabiliseKp += KpAdd;
  stabiliseKi += KiAdd;
  stabiliseKd += KdAdd;
  xStabilisePID.SetTunings(stabiliseKp, stabiliseKi, stabiliseKd);
  Serial.print("\n\r"); Serial.print(stabiliseKp*100); Serial.print("\t"); Serial.print(stabiliseKi*100); Serial.print("\t"); Serial.print(stabiliseKd*100); Serial.print("   /100");
}

void tuneRate(double KpAdd, double KiAdd, double KdAdd)
{
  rateKp += KpAdd;
  rateKi += KiAdd;
  rateKd += KdAdd;
  xRatePID.SetTunings(rateKp, rateKi, rateKd);
  Serial.print("\n\r"); Serial.print(rateKp*100); Serial.print("\t"); Serial.print(rateKi*100); Serial.print("\t"); Serial.print(rateKd*100); Serial.print("   /100");
}

void pidSetup()
{
  xStabilisePID.SetMode(AUTOMATIC);
  xStabilisePID.SetOutputLimits(-90,90);
  xStabilisePID.SetSampleTime(5);
  xRatePID.SetMode(AUTOMATIC);
  xRatePID.SetOutputLimits(-90,90);
  xRatePID.SetSampleTime(10);
}

void pidStage(float xActualAngle, float xActualRate, float xDesiredAngle, float* motorSpeeds)
{
  pidXActualAngle = xActualAngle;
  pidXActualRate = xActualRate;
  pidXDesiredAngle = xDesiredAngle;
  pidXDesiredRate = 0; //for testing
  
  if (xRatePID.Compute()) //xStabilisePID.Compute() && 
  {
    (motorSpeeds)[0] += pidXRateOutput;
    (motorSpeeds)[1] -= pidXRateOutput;
    (motorSpeeds)[2] += pidXRateOutput;
    (motorSpeeds)[3] -= pidXRateOutput;
    Serial.print("\n\r"); Serial.print(pidXRateOutput);
  }
}
