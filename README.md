# Balancing-Robot-V1.0
///THIS IS STILL A WORK IN PROGRESS 

Video presentation: https://youtu.be/kiIgf5LplvQ

Code for self balancing robot made by Heriberto Leyva Diaz de Leon. This code incorporates: 
- The DMP( Direct Motion Processing)  of the MPU 9250
- Self made PID control loop(to balance the robot) with in a PD control loop( to maintain a fixed position).
- Bluetooth control though the Android App: Joystick BT Commander
- Direct register manipulation for fast processing 
- Intterupt attachment for the encoders 

CAD files were made in the Fusion 360 software (it is free, FYI).

Bill of important materials: 
- Pololu 37D 30:1 motor with encoder
- Dual VNH 5019 motor driver
- MPU 9250 
- Arduino UNO
- 1/4 inch  threaded steel rods
- HC-06 Bluetooth module 
- Turnigy 5000 mAh 20C LIPO battery 
- Wild Thumper wheels from Pololu (GREATLY RECOMENDED)

Future work when this arduino part is at 100%, Raspberry Pi control with Open CV image processing. 

