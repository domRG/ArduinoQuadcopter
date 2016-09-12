#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "Wire.h"
#include "helper_3dmath.h"
MPU6050 mpu;
#include <Servo.h>
#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_WORLDACCEL
Servo frontLeft, frontRight, backLeft, backRight;

// ================================================================================
// ================================================================================
// =====MPU                                                                   =====
// ================================================================================
// ================================================================================

const int LED_PIN = 13; // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

float mpuReturn[] = {0,0,0,0,0,0}; //aX aY aZ gX(r) gY(p) gZ(y)

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// ========================================
// =====FUNCTIONS                     =====
// ========================================

void dmpDataReady() {
    mpuInterrupt = true;
}

void mpuBegin(){
  Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(503); //497
    mpu.setYGyroOffset(-50); //-53
    mpu.setZGyroOffset(-57); //-57
    mpu.setZAccelOffset(880); //879
    mpu.setXAccelOffset(-3150); //-3109
    mpu.setYAccelOffset(1136); //1139
    

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void mpuRunScript(){
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("\n\rFIFO overflow, resetting FIFO"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
       
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      Serial.print("ypr    ");
      Serial.print(ypr[0] * 180/M_PI);
      mpuReturn[5] = ypr[0] * 180/M_PI;
      Serial.print("  ");
      Serial.print(ypr[1] * 180/M_PI);
      mpuReturn[4] = ypr[1] * 180/M_PI;
      Serial.print("  ");
      Serial.print(ypr[2] * 180/M_PI);
      mpuReturn[3] = ypr[2] * 180/M_PI;
      Serial.print("  "); Serial.print(mpuReturn[3]); Serial.print("  "); Serial.print(mpuReturn[4]); Serial.print("  "); Serial.print(mpuReturn[5]);
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      //Serial.print("aworld    ");
      //Serial.print(aaWorld.x);
      //Serial.print("    ");
      //Serial.print(aaWorld.y);
      //Serial.print("    ");
      //Serial.println(aaWorld.z);
    #endif
  
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}

// ================================================================================
// ================================================================================
// =====SERVO CONTROL                                                         =====
// ================================================================================
// ================================================================================

int speeds[] = {0,0,0,0}; //fL,fR,bL,bR
int maxSpeed = 2400;
int minSpeed = 544;
int action;
bool crash = false;

// ========================================
// =====FUNCTIONS                     =====
// ========================================

// Begin servos
void servoBegin(){
  frontLeft.attach(10);
  frontRight.attach(9);
  backLeft.attach(5);
  backRight.attach(3);
  action = Serial.read();
  fL(0);
  fR(0);
  bL(0);
  bR(0);
  action = Serial.read();
  Serial.print("\n\n\rEnter '1' as the ESCs beep for full calibration of servos\n\rOr '2' to skip servo calibration");
  while(!(action == 49 || action == 50)){
    action = Serial.read();
  }
  if(action == 49){
    Serial.print("\n\rInitialising. Please wait...");
    //delay(2250);
    Serial.print("\n\rBeep");
    Serial.print("\n\rSet to HIGH");
    fL(2400);
    fR(2400);
    bL(2400);
    bR(2400);
    delay(1164);
    Serial.print("\n\rBeep Beep");
    Serial.print("\n\rSet to LOW");
    fL(0);
    fR(0);
    bL(0);
    bR(0);
    Serial.print("\n\rBeep Beep Beep");
  }
  if(action == 50){
    Serial.print("\n\rSkipping servo calibration...");
    fL(0);
    fR(0);
    bL(0);
    bR(0);
  }
  Serial.print("\n\rInitialisation complete. Begin");
  speeds[0] = speeds[1] = speeds[2] = speeds [3] = 500;
}

// Servo control declatation
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
void servoSpeed(){
  fL(speeds[0]);
  fR(speeds[1]);
  bL(speeds[2]);
  bR(speeds[3]);
}

// Balance
void balance(){ //back left as control (dont adjust)
  //positive roll over top from left to right
  //positive pitch over top from front to back
  if(mpuReturn[3] > 0.5){
    if(speeds[1] < 2400){
      speeds[1]+=1;
    }
    if(speeds[3] < 2400){
      speeds[3]+=1;
    }
  }
  else if(mpuReturn[3] < -0.5){
    if(speeds[1] > 500){
      speeds[1]-=1;
    }
    if(speeds[3] > 500){
      speeds[3]-=1;
    }
  }
  if(mpuReturn[4] > 0.5){
    if(speeds[0] > 500){
      speeds[0]-=1;
    }
    if(speeds[1] > 500){
      speeds[1]-=1;
    }
  }
  else if(mpuReturn[4] < -0.5){
    if(speeds[0] < 2400){
      speeds[0]+=1;
    }
    if(speeds[1] < 2400){
      speeds[1]+=1;
    }
  }
  servoSpeed();
}

void servoRunScript(){
  action = Serial.read();
  if(action == ' '){//"w"=119 "s"=115 " "=32
    speeds[0] = 0;
    speeds[1] = 0;
    speeds[2] = 0;
    speeds[3] = 0;
    crash = true;
    servoSpeed();
  }
  if(action == 'w' && crash == false){
    speeds[2]+=2;
  }
  if(action == 'o' && crash == false){
    speeds[2]+=100;
  }
  if(action == 's' && crash == false){
    speeds[2]-=2;
  }
  if(action == 'l' && crash == false){
    speeds[2]-=100;
  }
  if(crash == false){
    balance();
  }
  if(action == 'r'){
    crash = false;
    speeds[0] = speeds[1] = speeds[2] = speeds [3] = 500;
  }
  Serial.print("\n\r fL: "); Serial.print(speeds[0]); Serial.print(" fR: "); Serial.print(speeds[1]); Serial.print(" bL: "); Serial.print(speeds[2]); Serial.print(" bR: "); Serial.print(speeds[3]);
}
// ========================================================================================================================

// ================================================================================
// =====SETUP SCRIPT                                                          =====
// ================================================================================

void setup(){
    Serial.begin(115200);
    mpuBegin();
    Serial.print("\n\n\rBooting, please wait...");
    delay(10000);
    mpu.resetFIFO();
    Serial.print("\n\n\rBoot complete");
    servoBegin();
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        
        //servoRunScript();
        //run the servo script
    }

    servoRunScript();
    mpuRunScript();
    //run the mpu script
}
