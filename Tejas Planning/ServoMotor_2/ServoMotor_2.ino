/*
Into Robotics
*/
 
#include <Servo.h>  //add '<' and '>' before and after servo.h
 
int servoPin = 10;
 
Servo Xservo;  
Servo Yservo;  

 
int servoAngle = 0;   // servo position in degrees
 
void setup()
{
  Serial.begin(9600);  
  Yservo.attach(servoPin);
  Xservo.attach(8);
  Serial.println("Enter Angle");

}

char rx_byte = 0;
String rx_str = "";
boolean not_number = false;
int result;

void loop()
{
//control the servo's direction and the position of the motor

//   servo.write(45);      // Turn SG90 servo Left to 45 degrees
//   delay(5000);          // Wait 1 second
//   servo.write(90);      // Turn SG90 servo back to 90 degrees (center position)
//   delay(5000);          // Wait 1 second
//   servo.write(135);     // Turn SG90 servo Right to 135 degrees
//   delay(5000);          // Wait 1 second
//   servo.write(90);      // Turn SG90 servo back to 90 degrees (center position)
//   delay(5000);

//https://startingelectronics.org/software/arduino/learn-to-program-course/19-serial-input/

  if (Serial.available() > 0) {    // is a character available?
    rx_byte = Serial.read();       // get the character
    
    if ((rx_byte >= '0') && (rx_byte <= '9')) {
      rx_str += rx_byte;
    }
    else if (rx_byte == '\n') {
      // end of string
      if (not_number) {
        Serial.println("Not a number");
      }
      else {
        Serial.print("Angle: ");
        Serial.print(rx_str);
        result = rx_str.toInt(); //converts char to int! :D life is simple with C 

        Yservo.write(result);
        Xservo.write(result);
        
        Serial.println("");
        Serial.println("Enter Angle");
      }
      not_number = false;         // reset flag
      rx_str = "";                // clear the string for reuse
    }
    else {
      // non-number character received
      not_number = true;    // flag a non-number
    }
  } // end: if (Serial.available() > 0)
}

