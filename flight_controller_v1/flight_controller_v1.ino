/*
 * To Do:
 * MPU RATE mpuReturn[6,7,8] (xyz)
 */

#include <SoftwareSerial.h>

SoftwareSerial mySerial(11,12);//RX TX

float speeds[] = {0,0,0,0}; //fL,fR,bL,bR
float control[] = {0,0,0,0}; //rpyt
//float balancePrecision = 1;
int count = 0;
bool crash = true;
int action;
int highSpeed = 2000;
int lowSpeed = 1000;
int off = 0;
int runNumber = 1;
int crashAngle = 15;
int pid = 0;

void calibrateServos()
{
  action = Serial.read();
  Serial.print("\n\rPress X on controller to skip servo calibration, or press SELECT to calibrate servos.");
  while(!(action == 49 || action == 50)){
    action = Serial.read();
  }
  if(action == 49){
    Serial.print("\n\rCalibrating. Please wait...");
    //delay(2250);
    //mySerial.print("\n\rBeep");
    //mySerial.print("\n\rSet to HIGH");
    speeds[0] = speeds[1] = speeds[2] = speeds[3] = highSpeed;
    servoSetSpeeds(speeds);
    action = Serial.read();
    while(!(action == 49)){
      action = Serial.read();
    }
    //mySerial.print("\n\rBeep Beep");
    //mySerial.print("\n\rSet to LOW");
    speeds[0] = speeds[1] = speeds[2] = speeds[3] = off;
    servoSetSpeeds(speeds);
    //mySerial.print("\n\rBeep Beep Beep");
    crash = true;
  }
  if(action == 50){
    Serial.print("\n\rSkipping servo calibration...");
    speeds[0] = speeds[1] = speeds[2] = speeds[3] = off;
    crash = true;
  }
  Serial.print("\n\rInitialisation complete. Begin");
  crash = true;
}

void readActionUpdateSpeeds(){
  action = Serial.read();
  if(action == 'r')
  {
    crash = false;
    speeds[0] = speeds[1] = speeds[2] = speeds [3] = lowSpeed;
    control[3] = lowSpeed;
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
    tuneRate(0,0,0.1);
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
    tuneRate(0,0,-0.1);
  }
  if(crash == true){
    speeds[0] = speeds[1] = speeds[2] = speeds [3] = off;
    control[3] = off;
  }
  if(crash == false){
    switch (action){
      case ' ':
        speeds[0] = speeds[1] = speeds[2] = speeds [3] = off;
        crash = true;
        control[3] = off;
        break;
      case 't':
        control[3] += 5;
        break;
      case 'g':
        control[3] -= 5;
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
    }
  }
}

void clampSpeeds()
{
  if (!crash)
  {
    if(speeds[0] > highSpeed)
    {
    speeds[0] = highSpeed;
    }
    
    if(speeds[0] < lowSpeed)
    {
      speeds[0] = lowSpeed;
    }
    
    if(speeds[1] > highSpeed)
    {
      speeds[1] = highSpeed;
    }
    
    if(speeds[1] < lowSpeed)
    {
      speeds[1] = lowSpeed;
    }
    
    if(speeds[2] > highSpeed)
    {
      speeds[2] = highSpeed;
    }
    
    if(speeds[2] < lowSpeed)
    {
      speeds[2] = lowSpeed;
    }
    
    if(speeds[3] > highSpeed)
    {
      speeds[3] = highSpeed;
    }
    
    if(speeds[3] < lowSpeed)
    {
      speeds[3] = lowSpeed;
    }
  }
}

void unstableCrash(float* angles)
{
  if(angles[3]>crashAngle || angles[3]<-crashAngle || angles[4]>crashAngle || angles[4]<-crashAngle)
  {
    speeds[0] = speeds[1] = speeds[2] = speeds[3] = off;
    control[3] = off;
    crash = true;
  }
}

void setup()
{
  mySerial.begin(9600);
  Serial.begin(115200);
  delay(100);
  calibrateServos();
  mpuSetup();
  servoSetup();
  pidSetup();
  Serial.print("\n\rSetup complete");
  
}
//float x[] = {0,0,0,0,0,0,0,0,0};
//int lastLoop = -1;
//int loopTime = 0;
void loop()
{
  //int now1 = millis();
  //if (lastLoop >= 0)  loopTime += (now1 - lastLoop);
  //lastLoop = now1;
  float* result = mpuRunScript();
  if(!crash && control[3] >= (lowSpeed))
  {
    pidStage(result[3], result[6], control[0], result[4], result[7], control[1], speeds, control[3]); //void pidStage(float xActualAngle, float xActualRate, float xDesiredAngle, float* motorSpeeds)
  }
  readActionUpdateSpeeds(); //10/250 ms
  clampSpeeds();
  unstableCrash(result);
  servoSetSpeeds(speeds); //10/250 ms
  if(count++ >= 10)
  {
    count = 0;
    Serial.print("\n\rControl[3] = "); Serial.print(control[3]); Serial.print("\tMotor speeds = "); Serial.print(speeds[0]); Serial.print("\t"); Serial.print(speeds[1]); Serial.print("\t"); Serial.print(speeds[2]); Serial.print("\t"); Serial.print(speeds[3]);
    //Serial.print("\tresult[6,7,8] : "); Serial.print(result[6]); Serial.print("\t"); Serial.print(result[7]); Serial.print("\t"); Serial.print(result[8]);
    //loopTime = 0;
  }
}
