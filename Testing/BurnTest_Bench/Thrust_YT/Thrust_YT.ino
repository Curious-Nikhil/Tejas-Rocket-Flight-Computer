/*
  Model Rocket Engine Dynamometer - Nick Cinquino 9/1/15 - V.5 Original 10Kg loadcell
  Modified from Nathan Seidle, SparkFun Electronics, November 19th, 2014
  Output data in grams (g). There are 101.97 gram-force per Newton, for conversion.
  Your calibration factor may be very positive or very negative. It all depends on the
  setup of your individual loadcell, and the direction the sensors deflect from zero state.
  Need to experiment!
  Hack the HX711 board to set pin 15 high, for 80 samples per second.
  This example code uses bogde's excellent library: https://github.com/bogde/HX711
  bogde's library is released under a GNU GENERAL PUBLIC LICENSE
  Arduino pin 2 - HX711 Clock
  pin 3 - Serial Data In
  5V - VCC
  GND - GND
  Yellow LED on pin 13, igniter transistor/relay on pin 11, momentary pushbutton on pin
  12.
  Most any pin on the Arduino Uno will be compatible with DOUT/CLK.
  The HX711 board can be powered from 2.7V to 5V so the Arduino 5V power is fine.

*/
#include "HX711.h"
#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);
const int buttonPin = 12;  // start sequence button
const int ledPin = 13;     //LED indicator and/or buzzer
const int igniterPin = 11; //igniter transistor circuit
int buttonState = 0;
#define DOUT 3
#define CLK 2
HX711 scale(DOUT, CLK);
float calibration_factor = -560; //-560 works for my 10kg loadcell.
void setup()
{
  pinMode(buttonPin, INPUT);
  pinMode(igniterPin, OUTPUT);
  pinMode(ledPin, OUTPUT);
  Serial.begin(9600);
  Serial.println("HX711 Rocket Motor Dynamometer, V.5");
  Serial.println("Affix motor nozzle up. Place igniter in nozzle. Move away from test stand.");
  Serial.println("Press start button to initialize ignition sequence.");
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" MODEL ROCKET");
  lcd.setCursor(0, 1);
  lcd.print(" DYNAMOMETER");
  delay(2000);
  scale.set_scale();
  scale.tare(); //Reset the scale to 0
  long zero_factor = scale.read_average(); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale.
  Useful in permanent scale projects.
  Serial.println(zero_factor);
  Serial.println(" ");
}
void loop()
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(" Rocket Dyno");
  lcd.setCursor(0, 1);
  lcd.print(" STDBY ");
  scale.set_scale(calibration_factor);
  lcd.print(scale.get_units(), 1);
  lcd.print(" g");
  delay(500);
  buttonState = digitalRead(buttonPin);
  if (buttonState == HIGH)
  {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" Rocket Dyno");
    lcd.setCursor(0, 1);
    lcd.print(" STAND CLEAR!");
    Serial.println("IGNITION SEQUENCE ACTIVATED!");
    for (int i = 0; i <= 50; i++)
    {
      digitalWrite(ledPin, HIGH);
      delay(100);
      digitalWrite(ledPin, LOW);
      delay(100);
    }
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" Rocket Dyno");
    lcd.setCursor(1, 1);
    lcd.print(" AQUIRING DATA");
    digitalWrite(igniterPin, HIGH);
    Serial.print("Start time, ms: ");
    Serial.print(millis());
    Serial.println(" ");
    Serial.println();
    for (int i = 0; i <= 800; i++)
    { //800 samples at 80sa/sec = 10 seconds theoretical
      scale.set_scale(calibration_factor); //Adjust to the calibration factor
      Serial.print(scale.get_units(), 1);
      Serial.println();
    }
    Serial.println();
    Serial.print("Stop Time, ms: ");
    Serial.print(millis());
    digitalWrite(ledPin, LOW);
    digitalWrite(igniterPin, LOW);
    Serial.println();
  }
}
