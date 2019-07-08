#include <Servo.h>
#define mos 2
#define pushbutton A0
#define buzzer 7
#define LED 12
int s_pinX = 8;
int s_pinY = 10;

Servo servoX;
Servo servoY;

int servoAngle = 0;   // servo position in degrees

int startangle = 60;
int endangle = 120;

int centerangleX = 90;
int centerangleY = 90;

int buttonState = 0;

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

  pinMode(mos, OUTPUT);
  pinMode(pushbutton, INPUT);

  Serial.begin(9600);


  tone(buzzer, 2000, 300);
  delay(500);
  tone(buzzer, 3000, 300);
  tone(buzzer, 3500, 300);
  delay(500);
  tone(buzzer, 2500, 300);

}



void loop()
{

  buttonState = digitalRead(pushbutton);
  digitalWrite(mos, LOW); //Set Mos pin LOW, dont launch by accident!!

  Serial.println(buttonState);

  digitalWrite(LED, LOW);

  if (buttonState == HIGH) {
    tone(buzzer, 500, 500);
    digitalWrite(LED, HIGH);

    Serial.println("Starting a 10 second Timer!");
    delay(1000);
    digitalWrite(LED, LOW);


    tone(buzzer, 3500, 500);
    Serial.println("9");
    delay(1000);
    digitalWrite(LED, HIGH);


    tone(buzzer, 3500, 500);
    Serial.println("8");
    delay(1000);
    digitalWrite(LED, LOW);


    tone(buzzer, 3500, 500);
    Serial.println("7");
    delay(1000);
    digitalWrite(LED, LOW);


    tone(buzzer, 3500, 500);
    Serial.println("6");
    delay(1000);
    digitalWrite(LED, LOW);


    tone(buzzer, 3500, 500);

    Serial.println("5");
    delay(1000);
    digitalWrite(LED, HIGH);


    tone(buzzer, 3500, 500);
    Serial.println("4");
    delay(1000);
    digitalWrite(LED, LOW);

    tone(buzzer, 3500, 500);
    Serial.println("3");
    delay(1000);
    digitalWrite(LED, HIGH);

    tone(buzzer, 3500, 500);
    Serial.println("2");
    delay(1000);
    digitalWrite(LED, LOW);


    tone(buzzer, 3500, 500);

    Serial.println("1");
    delay(1000);
    digitalWrite(LED, HIGH);

    tone(buzzer, 3500, 500);
    Serial.println("0");
    delay(1000);
    digitalWrite(LED, LOW);

    tone(buzzer, 4500, 2000);
    Serial.println("Launch OFF!!");
    digitalWrite(LED, HIGH);
    
    Serial.println("ON");
    digitalWrite(mos, HIGH);
    
    delay(3000);

      // servo_fastsweep();

  }

  buttonState = 0; //Wait for switct again. RESET

  Serial.println("OFF");
  digitalWrite(mos, LOW);

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
