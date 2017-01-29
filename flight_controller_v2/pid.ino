#include <PID_v1.h>

//========== RPY pid declaration ==========

//Define Variables we'll be connecting to
double pidXDesiredAngle, pidXActualAngle, pidXDesiredRate, pidXActualRate, pidXRateOutput;
double pidYDesiredAngle, pidYActualAngle, pidYDesiredRate, pidYActualRate, pidYRateOutput;

//Specify the links and initial tuning parameters
double stabiliseKp = 0.00, stabiliseKi = 0.00, stabiliseKd = 0.00; //double stabiliseKp= 12.5, stabiliseKi= 2.00, stabiliseKd= 0.50;
double rateKp = 0.00, rateKi = 0.00, rateKd = 0.00; //0.0105; //double rateKp= 1.4, rateKi= 0.05, rateKd= 1.3;

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

//========== Rps/Throttle pid declaration ==========

double throttleKp = 0.10, throttleKi = 0.00, throttleKd = 0.00;

double pidDesiredThrottle[4];
double pidThrottleChange[4];
double pidActualRps[4];

PID throttle0PID(&pidActualRps[0], &pidThrottleChange[0], &pidDesiredThrottle[0], throttleKp, throttleKi, throttleKd, DIRECT);
PID throttle1PID(&pidActualRps[1], &pidThrottleChange[1], &pidDesiredThrottle[1], throttleKp, throttleKi, throttleKd, DIRECT);
PID throttle2PID(&pidActualRps[2], &pidThrottleChange[2], &pidDesiredThrottle[2], throttleKp, throttleKi, throttleKd, DIRECT);
PID throttle3PID(&pidActualRps[3], &pidThrottleChange[3], &pidDesiredThrottle[3], throttleKp, throttleKi, throttleKd, DIRECT);

void tuneThrottle(double KpAdd, double KiAdd, double KdAdd)
{
  throttleKp += KpAdd;
  throttleKi += KiAdd;
  throttleKd += KdAdd;
  throttle0PID.SetTunings(throttleKp, throttleKi, throttleKd);
  throttle1PID.SetTunings(throttleKp, throttleKi, throttleKd);
  throttle2PID.SetTunings(throttleKp, throttleKi, throttleKd);
  throttle3PID.SetTunings(throttleKp, throttleKi, throttleKd);
  Serial.print("\n\r"); Serial.print(throttleKp*100); Serial.print("\t"); Serial.print(throttleKi*100); Serial.print("\t"); Serial.print(throttleKd*100); Serial.print("   /100");
}

//========== pid Setup ==========

void pidSetup()
{
  Serial.print("\n\rPreparing PID");
  
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

  throttle0PID.SetMode(AUTOMATIC);
  throttle0PID.SetOutputLimits(-50,50);
  throttle0PID.SetSampleTime(10);
  throttle1PID.SetMode(AUTOMATIC);
  throttle1PID.SetOutputLimits(-50,50);
  throttle1PID.SetSampleTime(10);
  throttle2PID.SetMode(AUTOMATIC);
  throttle2PID.SetOutputLimits(-50,50);
  throttle2PID.SetSampleTime(10);
  throttle3PID.SetMode(AUTOMATIC);
  throttle3PID.SetOutputLimits(-50,50);
  throttle3PID.SetSampleTime(10);

  Serial.print("\n\rComplete");
}

//========== pid Run ==========

void pidStage_MPU(float xActualAngle, float xActualRate, float xDesiredAngle, float yActualAngle, float yActualRate, float yDesiredAngle, float desiredThrottle)
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
  if(xRatePID.Compute() || yRatePID.Compute())
  {
    pidDesiredThrottle[0] = desiredThrottle + pidXRateOutput + pidYRateOutput;
    pidDesiredThrottle[1] = desiredThrottle - pidXRateOutput + pidYRateOutput;
    pidDesiredThrottle[2] = desiredThrottle + pidXRateOutput - pidYRateOutput;
    pidDesiredThrottle[3] = desiredThrottle - pidXRateOutput - pidYRateOutput;
  }
    //Serial.print(pidActualThrottle); Serial.print("\t"); Serial.print(pidDesiredThrottle); Serial.print("\n\r");
}

void pidStage_RPS(float* motorSpeeds, float* currentRps) //needs actualRps and desiredThrottle (and motor speeds)
{
  pidActualRps[0] = currentRps[0];
  pidActualRps[1] = currentRps[1];
  pidActualRps[2] = currentRps[2];
  pidActualRps[3] = currentRps[3];
  if (throttle0PID.Compute())
  {
    motorSpeeds[0] += pidThrottleChange[0];
  }
  if(throttle1PID.Compute())
  {
    motorSpeeds[1] += pidThrottleChange[1];
  }
  if(throttle2PID.Compute())
  {
    motorSpeeds[2] += pidThrottleChange[2];
  }
  if(throttle3PID.Compute())
  {
    motorSpeeds[3] += pidThrottleChange[3];
  }
} 
