#include <PID_v1.h>

//Define Variables we'll be connecting to
double pidXDesiredAngle, pidXActualAngle, pidXDesiredRate, pidXActualRate, pidXRateOutput;
double pidYDesiredAngle, pidYActualAngle, pidYDesiredRate, pidYActualRate, pidYRateOutput;

//Specify the links and initial tuning parameters
double stabiliseKp= 12.5, stabiliseKi= 2.00, stabiliseKd= 0.50;
double rateKp= 0.011, rateKi= 0.00, rateKd= 0.01;

PID xStabilisePID(&pidXActualAngle, &pidXDesiredRate, &pidXDesiredAngle, stabiliseKp, stabiliseKi, stabiliseKd, DIRECT);
PID xRatePID(&pidXActualRate, &pidXRateOutput, &pidXDesiredRate, rateKp, rateKi, rateKd, DIRECT);
PID yStabilisePID(&pidYActualAngle, &pidYDesiredRate, &pidYDesiredAngle, stabiliseKp, stabiliseKi, stabiliseKd, REVERSE);
PID yRatePID(&pidYActualRate, &pidYRateOutput, &pidYDesiredRate, rateKp, rateKi, rateKd, REVERSE);

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
  xStabilisePID.SetSampleTime(10);
  xRatePID.SetMode(AUTOMATIC);
  xRatePID.SetOutputLimits(-90,90);
  xRatePID.SetSampleTime(10);
  
  yStabilisePID.SetMode(AUTOMATIC);
  yStabilisePID.SetOutputLimits(-90,90);
  yStabilisePID.SetSampleTime(10);
  yRatePID.SetMode(AUTOMATIC);
  yRatePID.SetOutputLimits(-90,90);
  yRatePID.SetSampleTime(10);
}

void pidStage(float xActualAngle, float xActualRate, float xDesiredAngle, float yActualAngle, float yActualRate, float yDesiredAngle, float* motorSpeeds)
{
  pidXActualAngle = xActualAngle;
  pidXActualRate = xActualRate;
  pidXDesiredAngle = xDesiredAngle;

  pidYActualAngle = yActualAngle;
  pidYActualRate = yActualRate;
  pidYDesiredAngle = yDesiredAngle;
  
  if (xStabilisePID.Compute() && xRatePID.Compute() && yStabilisePID.Compute() && yRatePID.Compute()) //xStabilisePID.Compute() && 
  {
    (motorSpeeds)[0] += pidXRateOutput;
    (motorSpeeds)[1] -= pidXRateOutput;
    (motorSpeeds)[2] += pidXRateOutput;
    (motorSpeeds)[3] -= pidXRateOutput;

    (motorSpeeds)[0] += pidYRateOutput;
    (motorSpeeds)[1] += pidYRateOutput;
    (motorSpeeds)[2] -= pidYRateOutput;
    (motorSpeeds)[3] -= pidYRateOutput;
    //Serial.print("\n\r"); Serial.print(pidXRateOutput);
  }
}
