#include <Servo.h>

Servo frontLeft, frontRight, backLeft, backRight;

//==========SERVO

// Begin servos
void servoSetup(){
  Serial.print("\n\rAssigning ports to servos");
  frontLeft.attach(10);
  frontRight.attach(9);
  backLeft.attach(5);
  backRight.attach(3);
  fL(off);
  fR(off);
  bL(off);
  bR(off);
  Serial.print("\n\rComplete");
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

