#include "HX711.h"
#include <SD.h>

#define mos 2
#define pushbutton A0
#define buzzer 7
#define LED 9

int buttonState = 0;
int i = 0;

//LOADCELL Stuff here
#define DOUT 4
#define CLK 5

HX711 scale;
float calibration_factor = -560;
float weight = 0;

//Timer
int time_ms;
unsigned long previousMillis;
unsigned long startMillis;
unsigned long currentMillis;
unsigned long time_millis;
unsigned long period = 1; // Time till Loadcell is active

//SD card
String filename;
File myFile;

void setup()
{
  Serial.begin(9600);
  Serial.println("Rocket Motor Burn Test Bench");

  pinMode(mos, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(pushbutton, INPUT);

  //Startup Sound
  tone(buzzer, 2000, 300);
  delay(500);
  tone(buzzer, 3000, 300);
  tone(buzzer, 3500, 300);
  delay(500);
  tone(buzzer, 2500, 300);
  delay(1000);

  //LOADCELL
  //power_down();
  //  scale.begin(DOUT, CLK);
  //
  //  scale.set_scale();
  //  scale.tare();
  //Serial.println("Loadcel is set up");

  //SD CARD initialization

  initializeSD();

  // if (scale.wait_ready_timeout(1000) == 0)
  // {
  //   Serial.println("HX711 not Found");
  //   RED();
  //   while (1);
  // }
}

void loop()
{
  Serial.println("IN LOOP");
  buttonState = digitalRead(pushbutton);
  digitalWrite(mos, LOW); //Set Mos pin LOW, dont launch by accident!!
  digitalWrite(LED, LOW);

  if (buttonState == HIGH)
  {

    tone(buzzer, 500, 500);
    digitalWrite(LED, HIGH);

    //open and check if file can be opened or not.

    myFile = SD.open(filename, FILE_WRITE);

    if (myFile == 0)
    {
      RED();
      Serial.println("File can't be opened - ERROR = 0");
      while (1);
    }

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
    Serial.println("0, INGITION SEQUENCE START");
    delay(100);
    digitalWrite(LED, LOW);

    tone(buzzer, 4500, 2000);
    Serial.println("Launch OFF!!");
    digitalWrite(LED, HIGH);

    Serial.println("ON - LAUNCH OFF!!!");
    digitalWrite(mos, HIGH);

    //Write data to SD card for 16 seconds
    while (i < 1500)
    {
      //Time Instance
      currentMillis = millis(); 

      if (currentMillis - previousMillis > period) {
        previousMillis = currentMillis;

        time_ms++;
      }

      Serial.println(time_ms);

      Serial.println("THRUST");
      i++;
      //      scale.set_scale(calibration_factor); //Adjust to the calibration factor
      //      Serial.print(scale.get_units(), 1);

      //write to sd card

      myFile.print(time_ms);
      myFile.print(",");
      myFile.println(weight);
    }

    myFile.close();

    delay(1000);

    tone(buzzer, 2000, 300);
    delay(500);
    tone(buzzer, 3000, 300);
    tone(buzzer, 3500, 300);
    delay(500);
    tone(buzzer, 2500, 300);
  }

  buttonState = 0;

  digitalWrite(mos, LOW);
}

//Create a new filename everytime.
boolean loadSDFile()
{
  int i = 0;
  boolean file = false;

  while (!file && i < 1024)
  {
    filename = (String)i + "TLP.txt";

    if (!SD.exists(filename))
    {
      myFile = SD.open(filename, FILE_WRITE);
      delay(10);
      myFile.close();
      file = true;
    }
    i++;
  }

  return file;
}

void initializeSD()
{

  Serial.print(F("Initializing SD card..."));

  if (!SD.begin(4))
  {
    Serial.println(F("initialization failed!"));
    RED();
    while (1);
  }
  Serial.println(F("initialization done."));

  //Create a file with new name
  if (!loadSDFile())
  {
    Serial.println("Failed to create file");
    RED();
    while (1);
  }
  else
  {
    Serial.println("File name created!");
  }

  Serial.println(filename);

  myFile = SD.open(filename, FILE_WRITE);
  Serial.println(myFile);
  if (myFile)
  {
    //Print Header Files - meters, pascal, est_alt, mpuPitch, mpuRoll, mpuYaw, OutputX, OutputY

    myFile.print("Time(ms)");
    myFile.print(",");
    myFile.println("weight");

    myFile.close();
    Serial.println(F("File Created and File Closed"));
  }
  else
  {
    Serial.print(F("Error while opening file"));
    RED();
    while (1);
  }
}

//Error Function
void RED()
{
  digitalWrite(LED, HIGH);
  tone(7, 2500, 100);
  delay(200);
  digitalWrite(LED, LOW);

  tone(7, 2500, 100);
  delay(200);
  digitalWrite(LED, HIGH);

  tone(7, 2000, 100);
  delay(500);
  tone(7, 2000, 100);
}
