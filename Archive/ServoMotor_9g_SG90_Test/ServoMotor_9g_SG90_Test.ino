/*
  Into Robotics
*/

#include <Servo.h>  //add '<' and '>' before and after servo.h

int s_pinX = 8;
int s_pinY = 10;

Servo servoX;
Servo servoY;

int servoAngle = 0;   // servo position in degrees

int startangle = 60;
int endangle = 120;

int centerangleX = 90;
int centerangleY = 90;

void setup()
{
  Serial.begin(9600);
  servoX.attach(s_pinX);
  servoY.attach(s_pinY);



  servoX.write(centerangleX);      // Turn SG90 servo Left to 45 degrees
  delay(2000);
  servoY.write(centerangleY);      // Turn SG90 servo Left to 45 degrees
  delay(2000);

  //servo_sweep();

}



void loop()
{
 // servo_fastsweep();
}



void servo_fastsweep() {
  //X *******************************************

  for (servoAngle = startangle; servoAngle < endangle; servoAngle += 2) //move the micro servo from 0 degrees to 180 degrees
  {

    servoX.write(servoAngle);
    delay(10);
  }

  delay(100);

  for (servoAngle = endangle; servoAngle > startangle; servoAngle -= 2) //now move back the micro servo from 0 degrees to 180 degrees
  {
    servoX.write(servoAngle);
    delay(10);
  }

  delay(100);
  servoX.write(centerangleX);
  delay(200);


  // for Y

  //****************************************

  for (servoAngle = startangle; servoAngle < endangle; servoAngle += 2) //move the micro servo from 0 degrees to 180 degrees
  {

    servoY.write(servoAngle);
    delay(10);
  }

  delay(100);

  for (servoAngle = endangle; servoAngle > startangle; servoAngle -= 2) //now move back the micro servo from 0 degrees to 180 degrees
  {
    servoY.write(servoAngle);
    delay(10);
  }
  delay(100);
  servoX.write(centerangleX);
  delay(200);
}

void servo_sweep() {

  //X *******************************************

  for (servoAngle = startangle; servoAngle < endangle; servoAngle++) //move the micro servo from 0 degrees to 180 degrees
  {

    servoX.write(servoAngle);
    delay(10);
  }

  delay(100);

  for (servoAngle = endangle; servoAngle > startangle; servoAngle--) //now move back the micro servo from 0 degrees to 180 degrees
  {
    servoX.write(servoAngle);
    delay(10);
  }

  delay(1000);


  servoX.write(centerangleX);

  // for Y

  //****************************************

  for (servoAngle = startangle; servoAngle < endangle; servoAngle++) //move the micro servo from 0 degrees to 180 degrees
  {

    servoY.write(servoAngle);
    delay(10);
  }

  delay(100);

  for (servoAngle = endangle; servoAngle > startangle; servoAngle--) //now move back the micro servo from 0 degrees to 180 degrees
  {
    servoY.write(servoAngle);
    delay(10);
  }

  delay(1000);

  servoX.write(centerangleX);
  servoY.write(centerangleX);
}
