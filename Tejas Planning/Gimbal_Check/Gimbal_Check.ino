#include <Servo.h>  //add '<' and '>' before and after servo.h

int spinX = 8;
int spinY = 10;
 
Servo servoX;  
Servo servoY;
/*
Prototype Config:
Y_centre = 65
X_centre = 45
*/
int servoAngle = 0;   // servo position in degrees
int sx_centre = 45;
int sy_centre = 70;
void setup()
{
  Serial.begin(9600);  
  servoX.attach(spinX);
  servoY.attach(spinY);

   //set to centre
   servoX.write(sx_centre);
   servoY.write(sy_centre);
}
 
 
void loop()
{     
   //Check in X - Axis
   servoX.write(90);
   delay(500);
   servoX.write(45);
   delay(500);
    servoX.write(0);
   delay(500);

   //Centre to avoid obstruction
   servoX.write(sx_centre);
   servoY.write(sy_centre);
   delay(1000);
   
   //Check in Y - Axis
   servoY.write(100);
   delay(500);
   servoY.write(60);
   delay(500);
   servoY.write(30);
   delay(500);  
   servoY.write(sy_centre);
   delay(5000); //5 seconds to start!
}
