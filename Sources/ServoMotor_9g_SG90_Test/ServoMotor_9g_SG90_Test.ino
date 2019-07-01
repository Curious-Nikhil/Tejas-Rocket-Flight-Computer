/*
Into Robotics
*/
 
#include <Servo.h>  //add '<' and '>' before and after servo.h

int spinX = 8;
int spinY = 10;
 
Servo servoX;  
Servo servoY;
 
int servoAngle = 0;   // servo position in degrees
 
void setup()
{
  Serial.begin(9600);  
  servoX.attach(spinX);
  servoY.attach(spinY);
}
 
 
void loop()
{
//control the servo's direction and the position of the motor

   servoX.write(45);      // Turn SG90 servo Left to 45 degrees
   delay(1000);          // Wait 1 second
   servoX.write(90);      // Turn SG90 servo back to 90 degrees (center position)
   delay(1000);          // Wait 1 second
   servoX.write(90);      // Turn SG90 servo back to 90 degrees (center position)
   delay(1000);

//end control the servo's direction and the position of the motor

//
////control the servo's speed  
//
//if you change the delay value (from example change 50 to 10), the speed of the servo changes
  for(servoAngle = 0; servoAngle < 90; servoAngle++)  //move the micro servo from 0 degrees to 180 degrees
  {                                  
    servoX.write(servoAngle);              
    delay(50);                  
  }

  for(servoAngle = 90; servoAngle > 0; servoAngle--)  //now move back the micro servo from 0 degrees to 180 degrees
  {                                
    servoX.write(servoAngle);          
    delay(10);      
  }

  //end control the servo's speed  




  //control the servo's direction and the position of the motor

   servoY.write(45);      // Turn SG90 servo Left to 45 degrees
   delay(1000);          // Wait 1 second
   servoY.write(90);      // Turn SG90 servo back to 90 degrees (center position)
   delay(1000);          // Wait 1 second
   servoY.write(90);      // Turn SG90 servo back to 90 degrees (center position)
   delay(1000);

//end control the servo's direction and the position of the motor

//
////control the servo's speed  
//
//if you change the delay value (from example change 50 to 10), the speed of the servo changes
  for(servoAngle = 0; servoAngle < 90; servoAngle++)  //move the micro servo from 0 degrees to 180 degrees
  {                                  
    servoY.write(servoAngle);              
    delay(50);                  
  }

  for(servoAngle = 90; servoAngle > 0; servoAngle--)  //now move back the micro servo from 0 degrees to 180 degrees
  {                                
    servoY.write(servoAngle);          
    delay(10);      
  }

  servoY.write(60);
  delay(1000);
}

