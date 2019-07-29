/** ***********************************************  
  For Code:
  https://github.com/chall2009/Arduino-Rocket-Launcher
  
 ****************************************************/

#include <Wire.h>

Adafruit_7segment segment = Adafruit_7segment();

//Potentiometer Setup
int potPin = 0;
int seconds = 0;
int h = 0;

//Button Setup
int inPin = 4;  // pushbutton connected to digital pin 4
int button = 1; // variable to store the read value

//Speaker Setup
int speakerPin = 12; //Peezo to digital pin 12

//Launch Pin Setup
int launchPin = 3; //connected to digital pin 3, HIGH closes circuit to ignitors so rocket goes boom!

void setup()
{
#ifndef __AVR_ATtiny85__
  Serial.begin(9600);

  //Button Setup
  pinMode(inPin, INPUT); // sets the digital pin 7 as input

  //Launch Pin Setup
  pinMode(launchPin, OUTPUT);

#endif
  segment.begin(0x70);

  //Play startup Sound
  tone(speakerPin, 700);
  delay(250);
  noTone(speakerPin);
  tone(speakerPin, 800);
  delay(250);
  noTone(speakerPin);
  tone(speakerPin, 600);
  delay(250);
  noTone(speakerPin);
}

void loop()
{
  if (button == 1)
  {
    //Pot  Low = 0, High = 1000
    int reading = analogRead(potPin);
    //Serial.println(reading);

    if (reading >= 0 && reading < 10)
    {
      seconds = 1;
    }
    if (reading >= 10 && reading < 20)
    {
      seconds = 2;
    }
    if (reading >= 20 && reading < 30)
    {
      seconds = 3;
    }
    if (reading >= 30 && reading < 40)
    {
      seconds = 4;
    }
    if (reading >= 40 && reading < 50)
    {
      seconds = 5;
    }
    if (reading >= 50 && reading < 60)
    {
      seconds = 6;
    }
    if (reading >= 60 && reading < 70)
    {
      seconds = 7;
    }
    if (reading >= 70 && reading < 80)
    {
      seconds = 8;
    }
    if (reading >= 80 && reading < 90)
    {
      seconds = 9;
    }
    if (reading >= 90 && reading < 100)
    {
      seconds = 10;
    }
    if (reading >= 100 && reading < 110)
    {
      seconds = 11;
    }
    if (reading >= 110 && reading < 120)
    {
      seconds = 12;
    }
    if (reading >= 120 && reading < 130)
    {
      seconds = 13;
    }
    if (reading >= 130 && reading < 140)
    {
      seconds = 14;
    }
    if (reading >= 140 && reading < 175)
    {
      seconds = 15;
    }
    if (reading >= 175 && reading < 250)
    {
      seconds = 30;
    }
    if (reading >= 250 && reading < 400)
    {
      seconds = 45;
    }
    if (reading >= 400 && reading < 600)
    {
      seconds = 60;
    }
    if (reading >= 600 && reading < 750)
    {
      seconds = 75;
    }
    if (reading >= 750 && reading < 900)
    {
      seconds = 90;
    }
    if (reading >= 900 && reading < 980)
    {
      seconds = 105;
    }
    if (reading >= 980)
    {
      seconds = 120;
    }

    //Format Display #:## for greater than 60 seconds
    if (seconds >= 60)
    {
      segment.drawColon(true);
      segment.writeDigitNum(0, 0);
      segment.writeDigitNum(1, (seconds / 60));          //minutes
      segment.writeDigitNum(3, (seconds % 60) / 10);     //tens of seconds
      segment.writeDigitNum(4, (((seconds % 60) % 10))); //seconds
      segment.writeDisplay();
    }

    //Format Display ##.# for less than 60 seconds but greater than 10 seconds
    if (seconds >= 10 && seconds < 60)
    {
      segment.drawColon(false);
      segment.writeDigitNum(0, (seconds / 10));
      segment.writeDigitNum(1, (seconds % 10), true);
      segment.writeDigitNum(3, 0);
      segment.writeDigitNum(4, 0);
      segment.writeDisplay();
    }
    //Format Display #.## for less than 10 seconds
    if (seconds < 10)
    {
      segment.drawColon(false);
      segment.writeDigitNum(0, 0);
      segment.writeDigitNum(1, seconds, true);
      segment.writeDigitNum(3, 0);
      segment.writeDigitNum(4, 0);
      segment.writeDisplay();
    }

    //segment.println(seconds);
    //segment.writeDisplay();

    //Button
    button = digitalRead(inPin); // read the input pin
                                 //Serial.println(button);

    delay(10);
  }

  if (button == 0)
  { //Code for Launch Button Pressed
    Serial.println("LAUNCH");

    //Play Sound
    tone(speakerPin, 700);
    delay(250);
    noTone(speakerPin);
    tone(speakerPin, 800);
    delay(250);
    noTone(speakerPin);

    h = seconds * 100;

    for (h != 0; h--;)
    {

      if (h >= 6000)
      {
        //Format Display ##:## for greater than 60sec
        segment.drawColon(true);
        segment.writeDigitNum(0, 0);                         //tens of minutes
        segment.writeDigitNum(1, (h / 6000));                //minutes
        segment.writeDigitNum(3, ((h % 6000) / 1000) % 10);  //tens of seconds
        segment.writeDigitNum(4, (((h % 6000 / 100) % 10))); //seconds
        segment.writeDisplay();
      }

      //Format Display ##.## for less than 60 seconds
      if (h < 6000)
      {
        segment.drawColon(false);
        segment.writeDigitNum(0, ((h % 6000) / 1000) % 10);        //tens seconds
        segment.writeDigitNum(1, (((h % 6000 / 100) % 10)), true); // seconds
        segment.writeDigitNum(3, (h % 100) / 10);                  // tenths of seconds
        segment.writeDigitNum(4, (h % 100) % 10);                  // hundredths of seconds
        segment.writeDisplay();
      }

      //Play sound last 10 seconds
      if (h <= 1000)
      {
        if (h == 1000)
        {
          tone(speakerPin, 800); //Play sound
        }
        if (h <= 1000 && (h % 100) / 10 == 0 && h > 50)
        {
          tone(speakerPin, 800); //Play sound
        }
        if ((h % 100) / 10 == 8)
        {
          noTone(speakerPin);
        }
      }

      //Play sound every 30 seconds (1:30, 1:00, 30.0)

      if (h / 100 == 30 || h / 100 == 60 || h / 100 == 90 && (h % 100) / 10 == 8)
      {
        tone(speakerPin, 800); //Play sound
      }

      if ((h % 100) / 10 == 8)
      {
        noTone(speakerPin);
      }

      delay(10);
    }

    //Launch Power to Igniter
    digitalWrite(launchPin, HIGH);
    tone(speakerPin, 900);        //Play sound
    segment.blinkRate(1);         //blink display
    delay(8000);                  //Send current to ignitor for 8 seconds
    digitalWrite(launchPin, LOW); //Turn Power off after 8 Seconds.
    noTone(speakerPin);
    //initialize
    delay(5000);
    segment.blinkRate(0);

    button = 1;
  }
}