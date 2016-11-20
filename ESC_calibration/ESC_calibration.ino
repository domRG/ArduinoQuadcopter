float speeds[] = {0,0,0,0};

int maxSpeed = 2400;
int minSpeed = 800;
int highSpeed = 2000;
int lowSpeed = 1000;
int off = 0;

int action;

#include <Servo.h>

Servo frontLeft, frontRight, backLeft, backRight;

//==========SERVO

// Begin servos
void servoSetup(){
  frontLeft.attach(10);
  frontRight.attach(9);
  backLeft.attach(5);
  backRight.attach(3);
  fL(off);
  fR(off);
  bL(off);
  bR(off);
  servoSetSpeeds(speeds);
}

void fL(int s){
  frontLeft.writeMicroseconds(s);
}
void fR(int s){
  frontRight.writeMicroseconds(s);
}
void bL(int s){
  backLeft.writeMicroseconds(s);
}
void bR(int s){
  backRight.writeMicroseconds(s);
}
void servoSetSpeeds(float* speeds){
  fL((int)speeds[0]);
  fR((int)speeds[1]);
  bL((int)speeds[2]);
  bR((int)speeds[3]);
}

void calibrateServos()
{
                            /* Controls
                             *       minSpeed | -10  | +10  | maxSpeed
                             *  bR :    1    ->  2  ->  3  ->   4
                             *  bL :    q    ->  w  ->  e  ->   r
                             *  fR :    a    ->  s  ->  d  ->   f
                             *  fL :    z    ->  x  ->  c  ->   v
                             *  
                             *  'space' = off
                             */
  action = Serial.read();
  switch (action)
  {
    case 'z':
      speeds[0] = minSpeed;
      break;
    case 'a':
      speeds[1] = minSpeed;
      break;
    case 'q':
      speeds[2] = minSpeed;
      break;
    case '1':
      speeds[3] = minSpeed;
      break;
    case 'x':
      speeds[0] -= 10;
      break;
    case 's':
      speeds[1] -= 10;
      break;
    case 'w':
      speeds[2] -= 10;
      break;
    case '2':
      speeds[3] -= 10;
      break;
    case 'c':
      speeds[0] += 10;
      break;
    case 'd':
      speeds[1] += 10;
      break;
    case 'e':
      speeds[2] += 10;
      break;
    case '3':
      speeds[3] += 10;
      break;
    case 'v':
      speeds[0] = maxSpeed;
      break;
    case 'f':
      speeds[1] = maxSpeed;
      break;
    case 'r':
      speeds[2] = maxSpeed;
      break;
    case '4':
      speeds[3] = maxSpeed;
      break;
    case ' ':
      speeds[0] = speeds[1] = speeds[2] = speeds[3] = off;
      break;
  }
}

void setup()
{
  Serial.begin(115200);
  servoSetup();
}

void loop()
{
  calibrateServos();
  Serial.print("\n\rSpeeds: "); Serial.print(speeds[0]); Serial.print("\t"); Serial.print(speeds[1]); Serial.print("\t"); Serial.print(speeds[2]); Serial.print("\t"); Serial.print(speeds[3]); Serial.print("\t");
  servoSetSpeeds(speeds);
}

