#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "Wire.h"
#include "helper_3dmath.h"
#include <SoftwareSerial.h>
#include <Servo.h>
#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_WORLDACCEL
Servo frontLeft, frontRight, backLeft, backRight;
MPU6050 mpu;
SoftwareSerial mySerial(11,12);//RX TX

// ================================================================================
// ================================================================================
// =====MPU                                                                   =====
// ================================================================================
// ================================================================================

const int LED_PIN = 13; // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

//getting delta-rotation variables
float prevRot[] = {0,0,0}; //gX(r) gY(p) gZ(y)
float mpuReturn[] = {0,0,0,0,0,0,0,0,0}; //aX aY aZ gX(r) gY(p) gZ(y) gRX(r) gRY(p) gRZ(y) +roll = left up right down -pitch = front down back up
float prevReadTime = 0;
float currentReadTime = 0;
float readDuration = 0;
float rotRate[] = {0,0,0};
float rotError[] = {0,0,0}; //rpy (xyz)
float control[] = {0,0,0,0};
float Kp = 0.5;
float Ki = 0.5;
float Kd = 1;
int runNumber = 1;
float speeds[] = {0,0,0,0}; //fL,fR,bL,bR
int maxSpeed = 2400;
int minSpeed = 900;
int action;
bool crash = true;
float balancePrecision = 0.1;
int count = 0;

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
    
    mySerial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    mySerial.println(F("Testing device connections..."));
    mySerial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    mySerial.println(F("Initializing DMP..."));
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
        mySerial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        mySerial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        mySerial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        mySerial.print(F("DMP Initialization failed (code "));
        mySerial.print(devStatus);
        mySerial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void mpuCalculateRate(){                          //float prevRot[] = {0,0,0}; //gX(r) gY(p) gZ(y)
                                                  //float mpuReturn[] = {0,0,0,0,0,0}; //aX aY aZ gX(r) gY(p) gZ(y)
                                                  //float prevReadTime = 0;
                                                  //float currentReadTime = 0;
                                                  //float readDuration = 0;
                                                  //float rotRate[] = {0,0,0};
  readDuration = currentReadTime - prevReadTime;
  mpuReturn[6] = (mpuReturn[3]-prevRot[0])/readDuration;
  prevRot[0] = mpuReturn[3];
  mpuReturn[7] = (mpuReturn[4]-prevRot[1])/readDuration;
  prevRot[1] = mpuReturn[4];
  mpuReturn[8] = (mpuReturn[5]-prevRot[2])/readDuration;
  prevRot[2] = mpuReturn[5];
  prevReadTime = currentReadTime;
  
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
    mySerial.println(F("\n\rFIFO overflow, resetting FIFO"));
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
      currentReadTime = millis() / 1000.0;
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      //mySerial.print("\typr\t\t");
      //mySerial.print(ypr[0] * 180/M_PI);
      mpuReturn[5] = ypr[0] * 180/M_PI;
      //mySerial.print("\t");
      //mySerial.print(ypr[1] * 180/M_PI);
      mpuReturn[4] = ypr[1] * 180/M_PI;
      //mySerial.print("\t");
      //mySerial.print(ypr[2] * 180/M_PI);
      mpuReturn[3] = ypr[2] * 180/M_PI;
      mpuCalculateRate();
      //mySerial.print("\tchange in time "); mySerial.print(readDuration);
      //mySerial.print("\trateRoll "); mySerial.print(mpuReturn[6]); mySerial.print("\tratePitch "); mySerial.print(mpuReturn[7]); mySerial.print("\trateYaw "); mySerial.print(mpuReturn[8]);
      
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      //mySerial.print("aworld    ");
      //mySerial.print(aaWorld.x);
      //mySerial.print("    ");
      //mySerial.print(aaWorld.y);
      //mySerial.print("    ");
      //mySerial.println(aaWorld.z);
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

// ========================================
// =====FUNCTIONS                     =====
// ========================================

// Begin servos
void servoBegin(){
  frontLeft.attach(10);
  frontRight.attach(9);
  backLeft.attach(5);
  backRight.attach(3);
  action = mySerial.read();
  fL(0);
  fR(0);
  bL(0);
  bR(0);
  action = mySerial.read();
  mySerial.print("\n\n\rEnter '1' as the ESCs beep for full calibration of servos\n\rOr '2' to skip servo calibration");
  while(!(action == 49 || action == 50)){
    action = mySerial.read();
  }
  if(action == 49){
    mySerial.print("\n\rInitialising. Please wait...");
    //delay(2250);
    mySerial.print("\n\rBeep");
    mySerial.print("\n\rSet to HIGH");
    fL(maxSpeed);
    fR(maxSpeed);
    bL(maxSpeed);
    bR(maxSpeed);
    delay(1164);
    mySerial.print("\n\rBeep Beep");
    mySerial.print("\n\rSet to LOW");
    fL(0);
    fR(0);
    bL(0);
    bR(0);
    mySerial.print("\n\rBeep Beep Beep");
    crash = true;
  }
  if(action == 50){
    mySerial.print("\n\rSkipping servo calibration...");
    fL(0);
    fR(0);
    bL(0);
    bR(0);
  }
  mySerial.print("\n\rInitialisation complete. Begin");
  crash = true;
}

// Servo control declatation
void fL(int s){
  frontLeft.writeMicroseconds(s);
}
void fR(int s){
  frontRight.writeMicroseconds(s);
}
void bL(float s){
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

//PID --> PI
void pidRot(){
  /*
   * error[x] = rot[x] - desired[x]
   * (5) = (5) - (0)
   * 1&3 increase + 5*0.5 
   * 
   */
   float xErrorI = 0;
   float yErrorI = 0;
   
   rotError[0] = mpuReturn[6] - control[0];
   if(rotError[0] < balancePrecision && rotError[0] > -balancePrecision){
    rotError[0] = 0;
   }
   
   xErrorI += rotError[0];
   
   speeds[1] += (int)(0.5 + rotError[0] * Kp) + (int)(0.5 + xErrorI * Ki);
   speeds[3] += (int)(0.5 + rotError[0] * Kp) + (int)(0.5 + xErrorI * Ki);
   
   rotError[1] = mpuReturn[7] - control[1];
   if(rotError[1] < balancePrecision && rotError[1] > -balancePrecision){
    rotError[1] = 0;
   }
   
   yErrorI += rotError[1];
   
   speeds[0] -= (int)(0.5 + rotError[1] * Kp) + (int)(0.5 + yErrorI * Ki);
   speeds[1] -= (int)(0.5 + rotError[1] * Kp) + (int)(0.5 + yErrorI * Ki);

  mySerial.print("\t"); mySerial.print(rotError[0]); mySerial.print(rotError[1]); mySerial.print(xErrorI); mySerial.print(yErrorI);
   
   if(speeds[0] > maxSpeed){
    speeds[0] = maxSpeed;
   }
   if(speeds[0] < minSpeed){
    speeds[0] = minSpeed;
   }
   if(speeds[1] > maxSpeed){
    speeds[1] = maxSpeed;
   }
   if(speeds[1] < minSpeed){
    speeds[1] = minSpeed;
   }
   if(speeds[2] > maxSpeed){
    speeds[2] = maxSpeed;
   }
   if(speeds[2] < minSpeed){
    speeds[2] = minSpeed;
   }
   if(speeds[3] > maxSpeed){
    speeds[3] = maxSpeed;
   }
   if(speeds[3] < minSpeed){
    speeds[3] = minSpeed;
   }
   servoSpeed();
}



// Balance Rotation
void balanceRot(){ //back left as control (dont adjust)
  //positive roll over top from left to right
  //positive pitch over top from front to back
  if(mpuReturn[3] > balancePrecision){
    if(speeds[1] < maxSpeed){
      speeds[1]+=1;
    }
    if(speeds[3] < maxSpeed){
      speeds[3]+=1;
    }
  }
  else if(mpuReturn[3] < -balancePrecision){
    if(speeds[1] > minSpeed){
      speeds[1]-=1;
    }
    if(speeds[3] > minSpeed){
      speeds[3]-=1;
    }
  }
  if(mpuReturn[4] > balancePrecision){
    if(speeds[0] > minSpeed){
      speeds[0]-=1;
    }
    if(speeds[1] > minSpeed){
      speeds[1]-=1;
    }
  }
  else if(mpuReturn[4] < -balancePrecision){
    if(speeds[0] < maxSpeed){
      speeds[0]+=1;
    }
    if(speeds[1] < maxSpeed){
      speeds[1]+=1;
    }
  }
  servoSpeed();
}

void servoRunScript(){
  action = mySerial.read();
  if(action == 'r'){
    crash = false;
    speeds[0] = speeds[1] = speeds[2] = speeds [3] = minSpeed;
  }
  if(crash == false){
    switch (action){
      case ' ':
        speeds[0] = 0;
        speeds[1] = 0;
        speeds[2] = 0;
        speeds[3] = 0;
        crash = true;
        servoSpeed();
        break;
      case 'w':
        speeds[2]+=5;
        break;
      case 's':
        speeds[2]-=5;
        break;
      case 'q':
        speeds[2]+=100;
        break;
      case 'a':
        speeds[2]-=100;
        break;
      case 't':
        runNumber = (runNumber + 1)%2;
        break;
      case 'u':
        Kp += 1;
        break;
      case 'j':
        Kp -= 1;
        break;
      case 'm':
        Kp += 0.5;
        break;
      case 'i':
        Ki += 1;
        break;
      case 'k':
        Ki -= 1;
        break;
      case ',':
        Ki += 0.5;
        break;
    }
  }

  if(crash == false){
    if(runNumber == 1){
      balanceRot();
      mySerial.print("\tROTATION"); mySerial.print("\tKp = "); mySerial.print(Kp); mySerial.print("\tKi = "); mySerial.print(Ki);
    }
    else if(runNumber == 0){
      pidRot();
      mySerial.print("\tPID"); mySerial.print("\tKp = "); mySerial.print(Kp); mySerial.print("\tKi = "); mySerial.print(Ki);
    }
  }
  
  if(mpuReturn[3]>15 || mpuReturn[3]<-15 || mpuReturn[4]>15 || mpuReturn[4]<-15){
    speeds[0] = 0;
    speeds[1] = 0;
    speeds[2] = 0;
    speeds[3] = 0;
    crash = true;
    servoSpeed();
  }
  mySerial.print("\n\r fL: "); mySerial.print(speeds[0]); mySerial.print(" fR: "); mySerial.print(speeds[1]); mySerial.print(" bL: "); mySerial.print(speeds[2]); mySerial.print(" bR: "); mySerial.print(speeds[3]);
}
// ========================================================================================================================

// ================================================================================
// =====SETUP SCRIPT                                                          =====
// ================================================================================

void setup(){
    mySerial.begin(9600);
    //mpuBegin();
    mySerial.print("\n\n\rBooting, please wait...");
    //delay(10000);
    //mpu.resetFIFO();
    mySerial.print("\n\n\rBoot complete");
    //servoBegin();
    backLeft.attach(5,900,2400);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {

    /*

    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {

        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        
        
    }

    servoRunScript(); //run the servo script
    mpuRunScript(); //run the mpu script

    */
    action = mySerial.read();
    if(action == 'r'){
      crash = false;
    }
    if(crash == false){
      switch(action){
        case ' ':
          crash = true;
          break;
        case 'w':
          speeds[0] += 100;
          break;
        case 's':
          speeds[0] -= 100;
          break;
        case 'q':
          speeds[0] += 5;
          break;
        case 'a':
          speeds[0] -= 5;
          break;
        case 'd':
          speeds[0] -= 0.25;
          break;
        case 'e':
          speeds[0] += 0.25;
          break;
        case 'f':
          speeds[0] = maxSpeed;
          break;
        case 'v':
          speeds[0] = minSpeed;
          break;
      }
    bL(speeds[0]);//meant to be bL
    }
    if(crash == true){
      speeds[0] = 0;
      bL(speeds[0]);//meant to be bL
    }
    mySerial.print(crash); mySerial.print("\t\t");
    mySerial.println(speeds[0]);
}

