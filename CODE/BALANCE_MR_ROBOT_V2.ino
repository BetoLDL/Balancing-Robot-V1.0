/*
    Copyright (C) 2017  Heriberto Leyva Díaz de León   
     
    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License <http://www.gnu.org/licenses/> for more details.
    
    Creators whose codes inspired this project: 
    Kristian Lauszus : http://blog.tkjelectronics.dk/2012/02/the-balancing-robot/
    Debra Ansell : http://www.geekmomprojects.com/mpu-6050-redux-dmp-data-fusion-vs-complementary-filter/
    Kas : http://forum.arduino.cc/index.php?topic=173246.0
 */

#include <Wire.h>

#include "MPU6050_6Axis_MotionApps20.h"

#define cbit(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit)) // Clear bit
#define sbit(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))  // Set bit

//Ports and pins used to decide the direction of the motors

#define DIRECTION_PORT PORTD
#define M1_ENA PD4
#define M1_ENB PD5
#define M2_ENA PD6
#define M2_ENB PD7

#define LEFT_MOTOR 9
#define RIGHT_MOTOR 10

#define    STX          0x02
#define    ETX          0x03
#define    SLOW         750                             // Datafields refresh rate (ms)
#define    FAST         250                             // Datafields refresh rate (ms)

byte cmd[8] = {0, 0, 0, 0, 0, 0, 0, 0};                 // bytes received
byte buttonStatus = 0;                                  // first Byte sent to Android device
long previousMillis = 0;                                // will store last time Buttons status was updated
long sendInterval = SLOW;                               // interval between Buttons status transmission (milliseconds)
String displayStatus = "xxxx";                          // message to Android device
int joyY, joyX;
int turning_speed = -70;
int max_speed = 80;


MPU6050 mpu;

// Encoder control variables
int loop_count = 0;
long wheel_position, last_wheel_position, velocity, target_position = 0, rightCount, leftCount;
bool stopped = false ;
bool encoders = false ;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// Orientation/motion variables
Quaternion q;
VectorFloat gravity;
//float euler[3];
float ypr[3];
float    GYRO_FACTOR;

// This global varible tells how to scale acclerometer data
float    ACCEL_FACTOR;

const float RADIANS_TO_DEGREES = 57.2958; //180/3.14159

// Timer values to rule control loop and angle integration
uint32_t now, then, timer, time_wait;
int sample_rate_us = 10000; 

// PID and control values
                     // set_point has to be gotten from the readings of the robot completely vertical and they will differ on accout of the mounting 
double last_error = 0, set_point = -.4, present_angle, Iterm = 0 , pwm_out; // Values that need to be remembered
double steer_offset = 0;
double Kp = 26 , Ki = 1 , Kd = 100 ; // Change these to change the PID // Kp = 40, Ki = 1 , Kd = 80 LAST GOOD FOR THIS CODE ( BIG WHEELS) // Kp = 33 , Ki = .75 , Kd = 70 ; LAST GOOD FOR THIS CODE ( SCOOTER WHEELS)
//-----------------------------------------------------------------------------------------------------------------------------
//---ENTER SETUP---------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
void setup() {
  // BLUETOOTH SETUP
  Serial.begin(57600); // Important baudrate in order to communicate with the app: Joystick Commander BT
  //END BLUETOOTH SETUP
  
  // SENSOR SETUP
    Wire.begin();
    TWBR = 12; // 400 KHz IIC bus frequency 
    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();
    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        //attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // Set the full scale range of the gyro
        uint8_t FS_SEL = 0;
        //mpu.setFullScaleGyroRange(FS_SEL);

        // get default full scale value of gyro - may have changed from default
        // function call returns values between 0 and 3
        uint8_t READ_FS_SEL = mpu.getFullScaleGyroRange();
        Serial.print("FS_SEL = ");
        Serial.println(READ_FS_SEL);
        GYRO_FACTOR = 131.0/(FS_SEL + 1);
        

        // get default full scale value of accelerometer - may not be default value.  
        // Accelerometer scale factor doesn't reall matter as it divides out
        uint8_t READ_AFS_SEL = mpu.getFullScaleAccelRange();
        Serial.print("AFS_SEL = ");
        Serial.println(READ_AFS_SEL);
        //ACCEL_FACTOR = 16384.0/(AFS_SEL + 1);
        
        // Set the full scale range of the accelerometer
        //uint8_t AFS_SEL = 0;
        //mpu.setFullScaleAccelRange(AFS_SEL);

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
  //END SENSOR SETUP


  //PIN SETUP
  attachInterrupt( 0 , leftEncoder , RISING ); // Left encoder
  attachInterrupt( 1 , rightEncoder, RISING ); // Right encoder
  
  pinMode( 4 , OUTPUT ); // Motor 1 direction
  pinMode( 5 , OUTPUT ); // Motor 1 direction
  pinMode( 6 , OUTPUT ); // Motor 2 direction
  pinMode( 7 , OUTPUT ); // Motor 2 direction
  pinMode( 9 , OUTPUT ); // Motor 1 PWM
  pinMode(10 , OUTPUT ); // Motor 2 PWM
  pinMode(13 , OUTPUT ); // Loop-time error
  //END PIN SETUP

  //PWM SETUP
  //PWM custom config straight from the datasheet
  TCCR1A = _BV(COM1A1) | _BV(COM1B1) | _BV(WGM12) | _BV(WGM11); // Activates COMA1 and COMB1 which are the PWM signals on pins 9 and 10
  TCCR1B = _BV(CS10); //WGM12 and 11 make the 10 bit fast PWM, and the CS10 is the preescaler of the timer which gives a 31.5 KHz signal
  //END PWM SETUP

  Serial.flush();
  
  //STARTING TIME KEEPING VARIABLES
  now = micros();
  timer = now;
}
//-----------------------------------------------------------------------------------------------------------------------------
//---ENTER LOOP----------------------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
void loop() {
  
  present_angle = getSensordata();

  bluetoothControl();

  pwm_out = pidCalc( present_angle );
  
  moveMotors( pwm_out, pwm_out );

  // Update the position and velocity of the robot every 10Hz
  loop_count++;
  if ( loop_count == 10 ){
    loop_count = 0 ;
    wheel_position = readLeftencoder() + readRightencoder() ; // Reads the encoders
    //Serial.print( " POSITION :  ");
    //Serial.println( wheel_position );
    //Serial.print( "    ");
    velocity = wheel_position - last_wheel_position; // Velocity calculation
    last_wheel_position = wheel_position; // Stores the position to get future velocity 
    if (  ( stopped == false ) ){ //When the robot is stopped the zero position is reset
      rightCount = 0;
      leftCount = 0; 
      stopped = true; 
    }
  }


  //FIXED TIME LOOP 
  then = micros();
  time_wait = sample_rate_us -( then - now);
  //Serial.println(time_wait);
  if ( time_wait > 0 ){ // Enough time for loop 
    PORTB = B000000;
  delayMicroseconds(time_wait); // Wait remaider of the time for constant sampling rate
  }
  else{ // If the time loop could not be completed because of not enough time, then signal with the built in LED
    PORTB = B100000;
  }
  now = micros();
  //END FIXED TIME LOOP
}
//-----------------------------------------------------------------------------------------------------------------------------
//---ENTER FUNCTION DECLARATION------------------------------------------------------------------------------------------------
//-----------------------------------------------------------------------------------------------------------------------------
void leftEncoder(){ // Interrupt routine for encoder, calculates the direction of rotation reading the direction pin of motor 1
  if ( PIND & _BV( PIND4 ) ) leftCount++;
  else leftCount--;
}
void rightEncoder(){ // Interrupt routine for encoder, calculates the direction of rotation reading the direction pin of motor 2
  if ( PIND & _BV( PIND6 ) ) rightCount++;
  else rightCount--;
}
long readLeftencoder(){ // Read the encoders
  return leftCount;
}
long readRightencoder(){ // Read the encoders
  return rightCount;
}
void bluetoothControl(){
  if(Serial.available())  { // data received from smartphone
   delayMicroseconds(1000);
   cmd[0] =  Serial.read();  
   if(cmd[0] == STX)  {
     int i=1;      
     while(Serial.available())  {
       //delay(1);
       cmd[i] = Serial.read();
       if(cmd[i]>127 || i>7)                 break; // Communication error
       if((cmd[i]==ETX) && (i==2 || i==7))   break; // Button or Joystick data
       i++;
     }
     if     (i==2)          getButtonState(cmd[1]);    // 3 Bytes  ex: < STX "C" ETX >  ADD BUTTONS IF YOU WANT TO 
     if(i==7)          getJoystickState(cmd);     // 6 Bytes  ex: < STX "200" "180" ETX >
   }
 }
}
void getJoystickState(byte data[8])    { // Interpreter of the joystick information 
 joyX = (data[1]-48)*100 + (data[2]-48)*10 + (data[3]-48); // obtain the Int from the ASCII representation
 joyY = (data[4]-48)*100 + (data[5]-48)*10 + (data[6]-48);
 joyX = joyX - 200; // Offset to avoid
 joyY = joyY - 200; // transmitting negative numbers

 if(joyX<-100 || joyX>100 || joyY<-100 || joyY>100)     return; // commmunication error

 if (joyY >= 0){ //Simple escaling, in the future the control loop can be implemented with three different  settings
  joyY = scale(joyY, 0, 100,0, 3);
 }else{
  joyY = 0 - joyY;
  joyY = scale(joyY, 0, 100,0, 3);
  joyY = 0 - joyY;
 }
 if (joyX >= 0){
  joyX  = scale(joyX, 0, 100,0, 3);
 }else{
  joyX = 0 - joyX;
  joyX = scale(joyX, 0, 100,0, 3);
  joyX = 0 - joyX;
 }
}
void getButtonState(int bStatus)  {
 switch (bStatus) {
// -----------------  BUTTON #1  -----------------------
   case 'A':
     encoders = true;
     break;
   case 'B':
     encoders = false;
     break;

// -----------------  BUTTON #2  -----------------------
   case 'C':
     buttonStatus |= B000010;        // ON
     Serial.println("\n** Button_2: ON **");
     // your code...      
     displayStatus = "Button2 <ON>";
     Serial.println(displayStatus);
     break;
   case 'D':
     buttonStatus &= B111101;        // OFF
     Serial.println("\n** Button_2: OFF **");
     // your code...      
     displayStatus = "Button2 <OFF>";
     Serial.println(displayStatus);
     break;

// -----------------  BUTTON #3  -----------------------
   case 'E':
     buttonStatus |= B000100;        // ON
     Serial.println("\n** Button_3: ON **");
     // your code...      
     displayStatus = "Motor #1 enabled"; // Demo text message
     Serial.println(displayStatus);
     break;
   case 'F':
     buttonStatus &= B111011;      // OFF
     Serial.println("\n** Button_3: OFF **");
     // your code...      
     displayStatus = "Motor #1 stopped";
     Serial.println(displayStatus);
     break;

// -----------------  BUTTON #4  -----------------------
   case 'G':
     buttonStatus |= B001000;       // ON
     Serial.println("\n** Button_4: ON **");
     // your code...      
     displayStatus = "Datafield update <FAST>";
     Serial.println(displayStatus);
     sendInterval = FAST;
     break;
   case 'H':
     buttonStatus &= B110111;    // OFF
     Serial.println("\n** Button_4: OFF **");
     // your code...      
     displayStatus = "Datafield update <SLOW>";
     Serial.println(displayStatus);
     sendInterval = SLOW;
    break;

// -----------------  BUTTON #5  -----------------------
   case 'I':           // configured as momentary button
//      buttonStatus |= B010000;        // ON
     Serial.println("\n** Button_5: ++ pushed ++ **");
     // your code...      
     displayStatus = "Button5: <pushed>";
     break;
//   case 'J':
//     buttonStatus &= B101111;        // OFF
//     // your code...      
//     break;

// -----------------  BUTTON #6  -----------------------
   case 'K':
     buttonStatus |= B100000;        // ON
     Serial.println("\n** Button_6: ON **");
     // your code...      
      displayStatus = "Button6 <ON>"; // Demo text message
    break;
   case 'L':
     buttonStatus &= B011111;        // OFF
     Serial.println("\n** Button_6: OFF **");
     // your code...      
     displayStatus = "Button6 <OFF>";
     break;
 }
// ---------------------------------------------------------------
}
double scale(double input, double inputMin, double inputMax, double outputMin, double outputMax) { // Like map() just returns a double
  double output;
  if(inputMin < inputMax)
    output = (input-inputMin)/((inputMax-inputMin)/(outputMax-outputMin));              
  else
    output = (inputMin-input)/((inputMin-inputMax)/(outputMax-outputMin));
  if(output > outputMax)
    output = outputMax;
  else if(output < outputMin)
    output = outputMin;
  return output;
}


void setPWM(uint8_t pin, double dutyCycle) { // Setting the duty cycle of the pins
  if ( pin == LEFT_MOTOR ){
    OCR1A = dutyCycle;
  }
  else if ( pin == RIGHT_MOTOR ){
    OCR1B = dutyCycle;
  }
}
double getSensordata(){ // Quaterninon angle calculation with the libraries 
    //mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;
        
        // Obtain Euler angles from buffer
        //mpu.dmpGetQuaternion(&q, fifoBuffer);
        //mpu.dmpGetEuler(euler, &q);
        
        // Obtain YPR angles from buffer
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        double current_angle = ypr[1] * RADIANS_TO_DEGREES;
        return current_angle;
        
        }
}

double pidCalc(double angle){
  if ( joyY > 0 ){ // Movement control forward/backward
    if ( steer_offset > -2.5 ) steer_offset -= 0.05;
    if ( pwm_out > (max_speed * -1) ) steer_offset -= 0.005;
    stopped = false ;
  }
  else if ( joyY < 0 ){ // Movement control forward/backward
    if ( steer_offset < 2.5 ) steer_offset += 0.05;
    if ( pwm_out < max_speed ) steer_offset += 0.005;
    stopped = false ;
  }
  else if ( encoders == true ){ // Position keeping PD controller 
    //long position_error = target_position - wheel_position;
    //Serial.print( wheel_position );
    steer_offset = -wheel_position * 0.0015 + velocity * 0.008 ;
    //Serial.print("   ");
    //Serial.println(steer_offset);
    //Serial.print("   ");
    //Serial.println(velocity );
  }else if ( encoders == false ){
    if ( steer_offset > 0.5 ) steer_offset -= 0.05;
    else if ( steer_offset < -0.05 ) steer_offset += 0.05;
    else steer_offset = 0;
  }
  if ( steer_offset > 5 ) steer_offset = 5; // Positive steer_offset contraint
  else if ( steer_offset < -5 ) steer_offset = -5; // Negative steer_offset constrait
  //Serial.print( " OFFSET:  ");
  //Serial.println( steer_offset );
  double error = ( set_point + steer_offset ) - angle ; // Error of the main PID control 
  Serial.println(error);
  if ( error < 30 && error > -30 ){ // beyond +-30 cut off motors for safety 
    double Pterm = Kp * error ;
    Iterm += Ki * error ;
    if ( Iterm >= 511 ) Iterm = 511 ;
    else if (Iterm <= -511 ) Iterm = -511;
    double Dterm = Kd * (error - last_error);
    last_error = error;
    double pid_out = Pterm + Iterm + Dterm ;
    return pid_out;
  }
  else{
    last_error = 0;
    Iterm = 0;
    return 0;
  }
}

void moveMotors(double pwm_left, double pwm_right ){ // Control of the Dual VNH 5019 from POLOLU
  if ( joyX > 0 ){
    //pwm_left += turning_speed;
    pwm_right -= turning_speed;
  }else if (joyX < 0 ){
    pwm_left -= turning_speed;
    //pwm_right += turning_speed;
  }
  if ( pwm_left >= 0 ){
    setPWM( LEFT_MOTOR   , pwm_left );
    cbit( DIRECTION_PORT , M1_ENA );
    sbit( DIRECTION_PORT , M1_ENB );
  }else{
    pwm_left = 0 - pwm_left;
    setPWM( LEFT_MOTOR   , pwm_left );
    sbit( DIRECTION_PORT , M1_ENA );
    cbit( DIRECTION_PORT , M1_ENB );
  }

  if ( pwm_right >= 0 ){
    setPWM( RIGHT_MOTOR  , pwm_right );
    cbit( DIRECTION_PORT , M2_ENA );
    sbit( DIRECTION_PORT , M2_ENB );
  }else{
    pwm_right = 0 - pwm_right;
    setPWM( RIGHT_MOTOR ,  pwm_right );
    sbit( DIRECTION_PORT , M2_ENA );
    cbit( DIRECTION_PORT , M2_ENB );
  }
}

