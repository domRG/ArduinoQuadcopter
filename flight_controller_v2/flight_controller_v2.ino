/*
 * To Do:
 * 
 */

#include <SoftwareSerial.h>

SoftwareSerial mySerial(11,12);//RX TX

float speeds[] = {0,0,0,0}; //fL,fR,bL,bR
float control[] = {0,0,0,0}; //rpyt
//float balancePrecision = 1;
int count = 0;
int action;
int highSpeed = 2400; //2000
int lowSpeed[] = {1550,1450,1500,1550}; //1000
int minRps = 20;
int maxRps = 70;
int off = 0;
int crashAngle = 30;

float* result = 0;

float inputValue = 0;

int runMode = 0; // 0 = Off, 1 = Idle, 2 = Running

//PID TUNING SCRIPT
int section = 0; // 0 == D, -25%, P, -50%, I, -50%, P, -0.2 == 7
unsigned long lastChangeTime = 0;

void tunePID()
{
  switch (section)
  {
    case 0:
      tuneRate(0,0,0.01);
      break;
    case 1:
      tune_d25();
      section++;
      break;
    case 2:
      tuneRate(0.2,0,0);
      break;
    case 3:
      tune_p50();
      section++;
      break;
    case 4:
      tuneRate(0,0.01,0);
      break;
    case 5:
      tune_i50();
      section++;
      break;
    case 6:
      tuneRate(0.1,0,0);
      break;
    case 7:
      tuneRate(-0.2,0,0);
      section++;
      break;
  }
}

void setRunModeStop()
{
  speeds[0] = speeds[1] = speeds[2] = speeds[3] = off;
  servoSetSpeeds(speeds);
  control[3] = off;
  runMode = 0;
}

void setRunModeIdle()
{
  speeds[0] = lowSpeed[0];
  speeds[1] = lowSpeed[1];
  speeds[2] = lowSpeed[2];
  speeds[3] = lowSpeed[3];
  servoSetSpeeds(speeds);
  control[3] = minRps;
  runMode = 1;
  Serial.println(runMode);
}

void setRunModePID()
{
  speeds[0] = lowSpeed[0];
  speeds[1] = lowSpeed[1];
  speeds[2] = lowSpeed[2];
  speeds[3] = lowSpeed[3];
  servoSetSpeeds(speeds);
  control[3] = minRps;
  runMode = 2;
  Serial.println(runMode);
}

void beginServos()
{
  Serial.print("\n\rBeginning Servos");
  setRunModeStop();
  Serial.print("\n\rServo initialisation complete");
}

void readActionUpdateSpeeds()
{
  action = mySerial.read(); //mySerial.read()
  if(action == 'r')
  {
    setRunModeIdle();
  }
  if(action == ' ')
  {
    setRunModeStop();
  }
  if(action == 'u')
  {
    tuneRate(0.2,0,0);
  }
  if(action == 'i')
  {
    tuneRate(0,0.01,0);
  }
  if(action == 'o')
  {
    tuneRate(0,0,0.01);
  }
  if(action == 'j')
  {
    tuneRate(-0.2,0,0);
  }
  if(action == 'k')
  {
    tuneRate(0,-0.01,0);
  }
  if(action == 'l')
  {
    tuneRate(0,0,-0.01);
  }
  if(runMode == 0)
  {
    setRunModeStop();
  }
  if(runMode == 1)
  {
    switch (action)
    {
      case 'p':
        setRunModePID();
        break;
    }
  }
  if(runMode == 2){
    switch (action)
    {
      case ' ':
        setRunModeStop();
        break;
      case 'b':
        inputValue = Serial.parseFloat();
        speeds[1] = inputValue;
        break;
      case 't':
        control[3] += 5;
        Serial.println("START");
        break;
      case 'g':
        control[3] -= 5;
        break;
      case 'y':
        control[3] = maxRps;
        break;
      case 'h':
        control[3] = minRps;
        break;
      case 'd':
        control[0] -= 5;
        break;
      case 'a':
        control[0] += 5;
        break;
      case 's':
        control[1] += 5;
        break;
      case 'w':
        control[1] -= 5;
        break;
      case 'm':
        section++;
        break;
    }
  }
}

void clampSpeeds()
{
  if(control[3] < minRps)
  {
    control[3] = minRps;
  }
  if(control[3] > maxRps)
  {
   control[3] = maxRps;
  }
  if(speeds[0] > highSpeed)
  {
  speeds[0] = highSpeed;
  }
  
  if(speeds[0] < lowSpeed[0])
  {
    speeds[0] = lowSpeed[0];
  }
  
  if(speeds[1] > highSpeed)
  {
    speeds[1] = highSpeed;
  }
  
  if(speeds[1] < lowSpeed[1])
  {
    speeds[1] = lowSpeed[1];
  }
  
  if(speeds[2] > highSpeed)
  {
    speeds[2] = highSpeed;
  }
  
  if(speeds[2] < lowSpeed[2])
  {
    speeds[2] = lowSpeed[2];
  }
  
  if(speeds[3] > highSpeed)
  {
    speeds[3] = highSpeed;
  }
  
  if(speeds[3] < lowSpeed[3])
  {
    speeds[3] = lowSpeed[3];
  }
}

void unstableCrash(float* angles)
{
  if(angles[3]>crashAngle || angles[3]<-crashAngle || angles[4]>crashAngle || angles[4]<-crashAngle)
  {
    setRunModeStop();
  }
}



void setup()
{
  mySerial.begin(9600);
  Serial.begin(250000);
  delay(100);
  beginServos();
  mpuSetup();
  servoSetup();
  pidSetup();
  Serial.print("\n\n\r=====Setup complete=====");
  setRunModeStop();
  delay(500);
}

unsigned long loopStart = 0;
unsigned long loopPrev = 0;

boolean isMPUValid(float * result) {
  return !(result[3] > -1e-2f && result[3] < 1e-2f &&
  result[4] > -1e-2f && result[4] < 1e-2f &&
  result[5] > -1e-2f && result[5] < 1e-2f);
}

void loop()
{
  readActionUpdateSpeeds(); //10/250 ms
  float* rotorRps = rps_loop();
  result = mpuRunScript();
  if (result != 0 && runMode == 2 && isMPUValid(result))
  {
    //Serial.print("\n\r\tMPU result[3,4,5] : "); Serial.print(result[3]); Serial.print("\t"); Serial.print(result[4]); Serial.print("\t"); Serial.print(result[5]); Serial.print("\t");
    pidStage_MPU(result[3], result[6], control[0], result[4], result[7], control[1], control[3]); //void pidStage(float xActualAngle, float xActualRate, float xDesiredAngle, float* motorSpeeds)
  }
  if(runMode == 2)
  {
    pidStage_RPS(speeds, rotorRps);
  }
  
  if (runMode == 1 || runMode == 2)
  {
    clampSpeeds();
  }
  unstableCrash(result);
  servoSetSpeeds(speeds); //10/250 ms
  if(count++ >= 39)
  {
    count = 0;
    //Serial.print(runMode); Serial.print("\t"); Serial.print(rotorRps[0]); Serial.print("\t"); Serial.print(rotorRps[1]); Serial.print("\t"); Serial.print(rotorRps[2]); Serial.print("\t"); Serial.println(rotorRps[3]);
    //Serial.print("\n\rControl[3] = "); Serial.print(control[3]);
    //Serial.print("\tMotor speeds = "); Serial.print(speeds[0]); Serial.print("\t"); Serial.print(speeds[1]); Serial.print("\t"); Serial.print(speeds[2]); Serial.print("\t"); Serial.println(speeds[3]);// Serial.print("\t");
    //Serial.print(control[3]); Serial.print("\t"); Serial.print(rotorRps[0]); Serial.print("\t"); Serial.print(rotorRps[1]); Serial.print("\t"); Serial.print(rotorRps[2]); Serial.print("\t"); Serial.print(rotorRps[3]); Serial.print("\t");
    loopStart = millis();
    //int loopDur = (loopStart - loopPrev) / 40L;
    //loopPrev = loopStart;
    //Serial.println(loopDur);
    
    if(runMode == 2 && (loopStart - lastChangeTime) >= 5000)
    {
      tunePID();
      lastChangeTime = loopStart;
    }
  }
  
}
