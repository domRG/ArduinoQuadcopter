# ArduinoQuadcopter
Project for building an Arduino controlled Quadcopter

## Description
This is a folder of the various stages I have made towards building an autonomous quadcopter using an Arduino Uno.
The aim is to have it completely self-controlled (so no remote control or radio antennas!),
with ambitions to later implement on-board collision avoidance and then self-navigation.

## Done and To-Do List:
- [x] Control of ESCs (and motors)
- [x] MPU to read rotation
- [ ] MPU to read rate (proving difficult)
- [x] MPU to calculate rate (work around but less accurate)
- [x] PID for roll
- [ ] PID for roll K values tuning
- [x] PID for pitch
- [ ] PID for pitch K values tuning
- [x] PID for yaw
- [ ] PID for yaw K values tuning
- [x] Throttle/Height control
- [x] Throttle PID (to prevent large jumps in motor speeds - causes issues)
- [x] Bluetooth communication (for remote debugging and Emergency Stop)
- [ ] Radio transmitter and reciever for longer range communication
- [ ] Distance sensors (aiming for at least 1 on each side (6 total)
- [ ] Collision avoidance

## Parts
- Arduino Uno
- MPU-6050
- HC-06
- Reptile 500 Frame
- 2S LiPo battery (x2)
- 1045 Propeller (x4)
- 1000 kV Motor (x4)
- 30 A ESC (x4)
- Power Distribution Board
