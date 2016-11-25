#include <PID_v1.h>

//========== RPY pid declaration ==========

//Define Variables we'll be connecting to
double pidXDesiredAngle, pidXActualAngle, pidXDesiredRate, pidXActualRate, pidXRateOutput;
double pidYDesiredAngle, pidYActualAngle, pidYDesiredRate, pidYActualRate, pidYRateOutput;

//Specify the links and initial tuning parameters
double stabiliseKp = 0.00, stabiliseKi = 0.00, stabiliseKd = 0.00; //double stabiliseKp= 12.5, stabiliseKi= 2.00, stabiliseKd= 0.50;
double rateKp = 0.00, rateKi = 0.00, rateKd = 0.0105; //double rateKp= 1.4, rateKi= 0.05, rateKd= 1.3;

PID xStabilisePID(&pidXActualAngle, &pidXDesiredRate, &pidXDesiredAngle, stabiliseKp, stabiliseKi, stabiliseKd, DIRECT);
PID xRatePID(&pidXActualRate, &pidXRateOutput, &pidXDesiredRate, rateKp, rateKi, rateKd, DIRECT);
PID yStabilisePID(&pidYActualAngle, &pidYDesiredRate, &pidYDesiredAngle, stabiliseKp, stabiliseKi, stabiliseKd, DIRECT);
PID yRatePID(&pidYActualRate, &pidYRateOutput, &pidYDesiredRate, rateKp, rateKi, rateKd, DIRECT);

void tuneStabilise(double KpAdd, double KiAdd, double KdAdd)
{
  stabiliseKp += KpAdd;
  stabiliseKi += KiAdd;
  stabiliseKd += KdAdd;
  xStabilisePID.SetTunings(stabiliseKp, stabiliseKi, stabiliseKd);
  yStabilisePID.SetTunings(stabiliseKp, stabiliseKi, stabiliseKd);
  Serial.print("\n\r"); Serial.print(stabiliseKp*100); Serial.print("\t"); Serial.print(stabiliseKi*100); Serial.print("\t"); Serial.print(stabiliseKd*100); Serial.print("   /100");
}

void tuneRate(double KpAdd, double KiAdd, double KdAdd)
{
  rateKp += KpAdd;
  rateKi += KiAdd;
  rateKd += KdAdd;
  xRatePID.SetTunings(rateKp, rateKi, rateKd);
  yRatePID.SetTunings(rateKp, rateKi, rateKd);
  Serial.print("\n\r"); Serial.print(rateKp*100); Serial.print("\t"); Serial.print(rateKi*100); Serial.print("\t"); Serial.print(rateKd*100); Serial.print("   /100");
}

//========== throttle pid declaration ==========

double pidDesiredThrottle, pidThrottleChange;
double pidActualThrottle = 1000; 

double throttleKp = 0.1, throttleKi = 0.00, throttleKd = 0.00;

PID throttlePID(&pidActualThrottle, &pidThrottleChange, &pidDesiredThrottle, throttleKp, throttleKi, throttleKd, DIRECT);

void tuneThrottle(double KpAdd, double KiAdd, double KdAdd)
{
  throttleKp += KpAdd;
  throttleKi += KiAdd;
  throttleKd += KdAdd;
  throttlePID.SetTunings(throttleKp, throttleKi, throttleKd);
  Serial.print("\n\r"); Serial.print(throttleKp*100); Serial.print("\t"); Serial.print(throttleKi*100); Serial.print("\t"); Serial.print(throttleKd*100); Serial.print("   /100");
}

//========== pid Setup ==========

void pidSetup()
{
  xStabilisePID.SetMode(AUTOMATIC);
  xStabilisePID.SetOutputLimits(-50,50);
  xStabilisePID.SetSampleTime(10);
  xRatePID.SetMode(AUTOMATIC);
  xRatePID.SetOutputLimits(-50,50);
  xRatePID.SetSampleTime(10);
  
  yStabilisePID.SetMode(AUTOMATIC);
  yStabilisePID.SetOutputLimits(-50,50);
  yStabilisePID.SetSampleTime(10);
  yRatePID.SetMode(AUTOMATIC);
  yRatePID.SetOutputLimits(-50,50);
  yRatePID.SetSampleTime(10);

  throttlePID.SetMode(AUTOMATIC);
  throttlePID.SetOutputLimits(-50,50);
  throttlePID.SetSampleTime(10);
}

//========== pid Run ==========

void pidStage(float xActualAngle, float xActualRate, float xDesiredAngle, float yActualAngle, float yActualRate, float yDesiredAngle, float* motorSpeeds, float desiredThrottle)
{
  pidXActualAngle = xActualAngle;
  pidXActualRate = xActualRate;
  pidXDesiredAngle = xDesiredAngle;
  pidXDesiredRate = 0; //for testing only

  if (pidXActualRate > -0.5 && pidXActualRate < 0.5)
  {
    pidXActualRate = 0;
  }

  pidYActualAngle = yActualAngle;
  pidYActualRate = yActualRate;
  pidYDesiredAngle = yDesiredAngle;
  pidYDesiredRate = 0; //for testing only

  if (pidYActualRate > -0.5 && pidYActualRate < 0.5)
  {
    pidYActualRate = 0;
  }

  pidDesiredThrottle = desiredThrottle;

  //Serial.print(pidActualThrottle); Serial.print("\t"); Serial.print(pidDesiredThrottle); Serial.print("\n\r");
  
  if (xRatePID.Compute() && yRatePID.Compute() && throttlePID.Compute()) //xStabilisePID.Compute() && xRatePID.Compute() && yStabilisePID.Compute() && yRatePID.Compute()
  {
    (motorSpeeds)[0] += pidThrottleChange + pidXRateOutput + pidYRateOutput;
    (motorSpeeds)[1] += pidThrottleChange - pidXRateOutput + pidYRateOutput;
    (motorSpeeds)[2] += pidThrottleChange + pidXRateOutput - pidYRateOutput;
    (motorSpeeds)[3] += pidThrottleChange - pidXRateOutput - pidYRateOutput;

    pidActualThrottle += pidThrottleChange;
    //Serial.print("   "); Serial.print(pidYActualRate); Serial.print("   "); Serial.println(pidYRateOutput);
  }
}
