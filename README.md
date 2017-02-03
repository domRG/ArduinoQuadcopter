# ArduinoQuadcopter
Project for building an Arduino controlled Quadcopter

## __Description__
  This is a folder of the various stages I have made towards building an autonomous quadcopter using an Arduino Uno. The aim is to have it completely self-controlled (so no remote control or radio antennas!), with ambitions to later implement on-board collision avoidance and then self-navigation.

## __Done and To-Do List:__
### _Components_
  - [x] Control of ESCs (and motors)
  - [x] Bluetooth communication (for remote debugging and Emergency Stop)
  - [x] Switch to Arduino Mega for more I/O Pins
  - [x] Fix/Implement Interrupt Pin with MPU6050 and Arduino Mega
  - [x] Replace ESCs (1 of the 4 was faulty, got different firmware so needed to replace all 4)
  - [ ] Radio transmitter and reciever for longer range communication

### _Sensors_
#### MPU
  - [x] MPU to read rotation
  - [x] MPU to calculate rate (work around but less accurate)
  
#### RPS
  - [x] Infrared digital encoder to measure RPS of each motor
  - [x] Adapt throttle PID to get motor RPS to set values
  
#### Ultrasonic
  - [ ] Distance sensors (aiming for at least 1 pointing down)
  - [ ] Collision avoidance

### _PIDs_

#### Roll
  - [x] PID for roll rotation
  - [x] PID for roll rate of rotation
  - [ ] PID for roll rotation K values tuning
  - [ ] PID for roll rate of rotation K values tuning

#### Pitch
  - [x] PID for pitch rotation
  - [x] PID for pitch rate of rotation
  - [ ] PID for pitch rotation K values tuning
  - [ ] PID for pitch rate of rotation K values tuning

#### Yaw
  - [ ] PID for yaw rotation
  - [ ] PID for yaw rate of rotation
  - [ ] PID for yaw rotation K values tuning
  - [ ] PID for yaw rate of rotation K values tuning

#### Throttle
  - [x] Throttle/Height control
  - [x] Throttle PID (to prevent large jumps in motor speeds - causes issues)

## __Parts__
  - Arduino Mega
  - MPU-6050
  - HC-06
  - Reptile 500 Frame
  - 2S LiPo battery (x2)
  - 1045 Propeller (x4)
  - 1000 kV Motor (x4)
  - 30 A ESC (x4)
  - Power Distribution Board
  - Infrared sensors
