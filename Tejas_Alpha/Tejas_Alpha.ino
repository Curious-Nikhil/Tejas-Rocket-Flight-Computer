// =============================
// == PHASE ZERO & ONE Devs ====
//==============================

/*Tejas Alpha - @Nikhil Mishra
 * curious-nikhil.github.io
 * Tejas is a rocket computer  
 * Tejas controls the pitch and the roll of the rocket
 */  

// =============================
// == Include and Define Vars ==
//==============================
//Header Files

#include <Servo.h>
#include "I2Cdev.h"
//#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"
#include "lib/PID/PID_v1.h"

//Vars and Defines

#define LED_PIN 13
bool blinkState = true;

//Servos 
Servo ServoX;
Servo ServoY;

//MPU Euler Angles
float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;

//Initiate MPU object
//MPU6050 mpu;


void setup() {
  //runs only onces

  ServoX.attach(8);
  ServoY.attach(10);

  //Reset Servos to 90 Deg
  ServoX.write(90);
  ServoY.write(90);

  Wire.begin();
}

void loop() {
  //runs repeatedly!
  
}

