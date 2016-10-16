/*
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file
#include "Wire.h"
#include "helper_3dmath.h"

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_READABLE_WORLDACCEL

float mpuReturn[] = {0,0,0,0,0,0,0,0,0}; //aX aY aZ gX(r) gY(p) gZ(y) gRX(r) gRY(p) gRZ(y) +roll = left up right down -pitch = front down back up
float mpuRotRate[] = {0,0,0};
bool blinkState = false;
const int LED_PIN = 13; // (Arduino is 13, Teensy is 11, Teensy++ is 6)
float prevRot[] = {0,0,0}; //gX(r) gY(p) gZ(y)
float prevReadTime = 0;
float currentReadTime = 0;

//==========MPU

MPU6050 mpu;

// MPU control/status vars
bool mpuDmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t mpuDevStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t mpuPacketSize;    // expected DMP packet size (default is 42 bytes)
uint16_t mpuFifoCount;     // count of all bytes currently in FIFO
uint8_t mpuFifoBuffer[64]; // FIFO storage buffer
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

void dmpDataReady() {
    mpuInterrupt = true;
}

void mpuSetup(){
  Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    
    //mySerial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    //mySerial.println(F("Testing device connections..."));
    //mySerial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    //mySerial.println(F("Initializing DMP..."));
    mpuDevStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(503); //497
    mpu.setYGyroOffset(-50); //-53
    mpu.setZGyroOffset(-57); //-57
    mpu.setZAccelOffset(880); //879
    mpu.setXAccelOffset(-3150); //-3109
    mpu.setYAccelOffset(1136); //1139
    

    // make sure it worked (returns 0 if so)
    if (mpuDevStatus == 0) {
        // turn on the DMP, now that it's ready
        //mySerial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //mySerial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //mySerial.println(F("DMP ready! Waiting for first interrupt..."));
        mpuDmpReady = true;

        // get expected DMP packet size for later comparison
        mpuPacketSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        //mySerial.print(F("DMP Initialization failed (code "));
        //mySerial.print(mpuDevStatus);
        //mySerial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
}

void mpuCalculateRate(){
  float readDuration = currentReadTime - prevReadTime;
  mpuReturn[6] = (mpuReturn[3]-prevRot[0])/readDuration;
  prevRot[0] = mpuReturn[3];
  mpuReturn[7] = (mpuReturn[4]-prevRot[1])/readDuration;
  prevRot[1] = mpuReturn[4];
  mpuReturn[8] = (mpuReturn[5]-prevRot[2])/readDuration;
  prevRot[2] = mpuReturn[5];
  prevReadTime = currentReadTime;
}

float* mpuRunScript(){
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  mpuFifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || mpuFifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    //mySerial.println(F("\n\rFIFO overflow, resetting FIFO"));
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (mpuFifoCount < mpuPacketSize) mpuFifoCount = mpu.getFIFOCount();
    // read a packet from FIFO
    mpu.getFIFOBytes(mpuFifoBuffer, mpuPacketSize);
       
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    mpuFifoCount -= mpuPacketSize;

    
    
    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      currentReadTime = millis() / 1000.0;
      mpu.dmpGetQuaternion(&q, mpuFifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
      mpuReturn[5] = ypr[0] * 180/M_PI;
      mpuReturn[4] = ypr[1] * 180/M_PI;
      mpuReturn[3] = ypr[2] * 180/M_PI;
      mpuCalculateRate();
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, mpuFifoBuffer);
      mpu.dmpGetAccel(&aa, mpuFifoBuffer);
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
  return mpuReturn;
}
*/
