/*
 * To Do:
 * test => fix rotErrors calculations
 */

#include <SoftwareSerial.h>

SoftwareSerial mySerial(11,12);//RX TX

float speeds[] = {0,0,0,0}; //fL,fR,bL,bR
float averageServoSpeed;
float control[] = {0,0,0,0}; //rpyt
//float balancePrecision = 1;
int count = 0;
bool crash = true;
int action;
int highSpeed = 2400;
int lowSpeed = 1000;
int off = 0;
int runNumber = 1;

void getAction()
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
    action = Serial.read();
    while(!(action == 49)){
      action = Serial.read();
    }
    //mySerial.print("\n\rBeep Beep");
    //mySerial.print("\n\rSet to LOW");
    speeds[0] = speeds[1] = speeds[2] = speeds[3] = off;
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

void avgSpeed(){
  averageServoSpeed = (float)(speeds[0] + speeds[1] + + speeds[2] + speeds[3]) / 4;
  if(averageServoSpeed > control[3]){
    speeds[0] -= 2;
    speeds[1] -= 2;
    speeds[2] -= 2;
    speeds[3] -= 2;
  }
  if(averageServoSpeed < control[3]){
    speeds[0] += 2;
    speeds[1] += 2;
    speeds[2] += 2;
    speeds[3] += 2;
  }
}

void readActionUpdateSpeeds(){
  avgSpeed();
  action = Serial.read();
  if(action == 'r'){
    crash = false;
    speeds[0] = speeds[1] = speeds[2] = speeds [3] = lowSpeed;
    control[3] = 1000;
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
      case 'w':
        control[3] += 10;
        break;
      case 's':
        control[3] -= 5;
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
  if(angles[3]>15 || angles[3]<-15 || angles[4]>15 || angles[4]<-15)
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
  getAction();
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
  if(!crash)
  {
    pidStage(result[3], speeds, control[0]);
  }
  readActionUpdateSpeeds(); //10/250 ms
  clampSpeeds();
  unstableCrash(result);
  servoSetSpeeds(speeds); //10/250 ms
  if(count++ >= 0)
  {
    count = 0;
    Serial.print("\n\rControl[3] = "); Serial.print(control[3]); Serial.print("\tMotor speeds = "); Serial.print(speeds[0]); Serial.print("\t"); Serial.print(speeds[1]); Serial.print("\t"); Serial.print(speeds[2]); Serial.print("\t"); Serial.print(speeds[3]); Serial.print("\t"); Serial.print(result[3]); Serial.print("\t"); Serial.print(result[4]); Serial.print("\t"); Serial.print(result[5]);
    //Serial.print("\tTime: "); Serial.print(loopTime);
    //loopTime = 0;
  }
}
