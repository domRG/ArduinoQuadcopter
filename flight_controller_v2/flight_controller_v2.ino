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
int lowSpeed[] = {1400,1300,1350,1400}; //1000
int minRps = 40;
int maxRps = 125;
int off = 0;
int crashAngle = 15;

int runMode = 0; // 0 = Off, 1 = Idle, 2 = Running

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
}

void beginServos()
{
  Serial.print("\n\rBeginning Servos");
  setRunModeStop();
  Serial.print("\n\rServo initialisation complete");
}

void readActionUpdateSpeeds(){
  action = Serial.read(); //mySerial.read()
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
    tuneRate(0.001,0,0);
  }
  if(action == 'i')
  {
    tuneRate(0,0.01,0);
  }
  if(action == 'o')
  {
    tuneRate(0,0,0.001);
  }
  if(action == 'j')
  {
    tuneRate(-0.001,0,0);
  }
  if(action == 'k')
  {
    tuneRate(0,-0.01,0);
  }
  if(action == 'l')
  {
    tuneRate(0,0,-0.001);
  }
  if(runMode == 0){
    setRunModeStop();
  }
  if(runMode == 1){
    switch (action){
      case 'p':
        setRunModePID();
        break;
    }
  }
  if(runMode == 2){
    switch (action){
      case ' ':
        setRunModeStop();
        break;
      case 't':
        float val = Serial.parseFloat()
        control[3] = val;
        break;
        //control[3] += 10;
        //Serial.println("START");
        //break;
      //case 'g':
        //control[3] -= 10;
        //break;
      //case 'y':
        //control[3] = maxRps;
        //break;
      //case 'h':
        //control[3] = minRps;
        //break;
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
    }
  }
}

void clampSpeeds()
{
  if (runMode != 0)
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

void loop()
{
  //loopStart = micros();
  //int loopDur = loopStart - loopPrev;
  //loopPrev = loopStart;
  readActionUpdateSpeeds(); //10/250 ms
  float* result = mpuRunScript();
  double* rotorRps = rps_loop();
  if(runMode == 2) //lowSpeed
  {
    pidStage(result[3], result[6], control[0], result[4], result[7], control[1], speeds, control[3]); //void pidStage(float xActualAngle, float xActualRate, float xDesiredAngle, float* motorSpeeds)
  }
  clampSpeeds();
  //unstableCrash(result);
  servoSetSpeeds(speeds); //10/250 ms
  if(count++ >= 0)
  {
    count = 0;
    //Serial.print("\n\rControl[3] = "); Serial.print(control[3]); Serial.print("\tMotor speeds = "); Serial.print(speeds[0]); Serial.print("\t"); Serial.print(speeds[1]); Serial.print("\t"); Serial.print(speeds[2]); Serial.print("\t"); Serial.print(speeds[3]); Serial.print("\t");
    Serial.print(control[3]); Serial.print("\t"); Serial.print(rotorRps[0]); Serial.print("\t"); Serial.print(rotorRps[1]); Serial.print("\t"); Serial.print(rotorRps[2]); Serial.print("\t"); Serial.print(rotorRps[3]); Serial.print("\t");
    //Serial.print("\n\r\tresult[6,7,8] : "); Serial.print(result[6]); Serial.print("\t"); Serial.print(result[7]); Serial.print("\t"); Serial.print(result[8]);
  }
  //Serial.println(loopDur);
}
