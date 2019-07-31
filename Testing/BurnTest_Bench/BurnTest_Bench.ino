#include "HX711.h"
#include <SD.h>

#define mos 2
#define pushbutton 5
#define buzzer 7
#define LED 9 // Red Light
#define GLED 8

//For Debug:
//#define MillisCount
//#define LOADCELL_CHECK

int buttonState = 0;
int i = 0;
bool Fire_State = 0;
bool Hold = 0;

//LOADCELL Stuff here
#define DOUT 2
#define CLK 3

HX711 scale;
float calibration_factor = 239;
float weight = 0;

//Timer
int time_ms;
unsigned long previousMillis;
unsigned long startMillis;
unsigned long currentMillis;
unsigned long period = 1; // Time till Loadcell is active

//SD card
String filename;
File myFile;

void setup()
{
  Serial.begin(9600);
  Serial.println("Rocket Motor Burn Test Bench");
  Serial.println("Checking Systems:");

  pinMode(mos, OUTPUT);
  pinMode(LED, OUTPUT);
  pinMode(pushbutton, INPUT_PULLUP);
  //attachInterrupt(5, ABORT, RISING);


  //Startup Sound
  tone(buzzer, 2000, 300);
  delay(500);
  tone(buzzer, 3000, 300);
  tone(buzzer, 3500, 300);
  delay(500);
  tone(buzzer, 2500, 300);
  delay(1000);

  //SD CARD initialization

  initializeSD();

  //Check if Loadcel is working.
  scale.begin(DOUT, CLK);

  scale.set_scale();
  scale.tare();
  long reading;
  Serial.println("Loadcel is set up");

  #ifdef LOADCELL_CHECK
  Serial.println(scale.read());
  if (scale.wait_ready_timeout(1000) == 0 || scale.read() < 1000)
  {
    Serial.println("HX711 not Found");
    RED();
    while (1);
  }
  scale.power_down();
  #endif

  //check if Fire Switch is enabled?
  buttonState = digitalRead(pushbutton);
  if (buttonState == 0) {
    //Error state, need to switch it off
    Serial.println("Fire Enable: Error");
    RED();
    while(1);
  }
}

void loop()
{
  buttonState = digitalRead(pushbutton);

  digitalWrite(mos, LOW); //Set Mos pin LOW, dont launch by accident!!

  GREEN_Idle();

  //1 - OFF, 0 - ON. - using INPUT_PULLUP.
  if (buttonState == 0)
  {
    Fire_State = 1;
    tone(buzzer, 500, 500);
    digitalWrite(LED, HIGH);

    myFile = SD.open(filename, FILE_WRITE);

    scale.power_up();
    scale.set_scale();
    scale.tare();

    //Check for file and loadcell error
    #ifdef LOADCELL_CHECK
    if (myFile == 0 || scale.read() < 1000)
    {
      RED();
      Serial.println("File or Loadcel - ERROR = 0");
      while (1);
    }

    Serial.println("Loadcel and SD Mod is set up");
    #endif

    #ifdef MillisCount
    unsigned long time_interval = 1000;
    int i;
    currentMillis = millis();

    for (i = 1; i <= 11; i++) {
      if (currentMillis - previousMillis >= i*time_interval) {
        previousMillis = currentMillis;

        if (i == 0) {
        Serial.println("Starting a 10 second Timer!");
        Serial.println(11 - i);
        tone(buzzer, 2500, 500);
        digitalWrite(LED, HIGH);
          
        }
        else if(i%2 == 0) {
            //If even
        digitalWrite(LED, HIGH);
        Serial.println(11 - i);
        tone(buzzer, 3500, 500);
        } 
        else {
        digitalWrite(LED, LOW);
        Serial.println(11 - i);
        tone(buzzer, 2500, 500);
        }
      }
    }

    #endif
    
    #define OLD
    #ifdef OLD
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
    digitalWrite(LED, HIGH);

    #endif

    buttonState = digitalRead(pushbutton); //check if Fire Button is still "ON"

    #define FIRE_PROGRAM
    #ifdef FIRE_PROGRAM
    if (buttonState == 0) {
      Serial.println("ON - LAUNCH OFF!!!");
      digitalWrite(mos, HIGH);

      //Write data to SD card for 16 seconds
      previousMillis = 0;

      while (i < 500)
      {
        Serial.println("While Loop");
        //Time Instance
        currentMillis = millis();

        if (currentMillis - previousMillis >= period) {
          previousMillis = currentMillis;

          time_ms++;  //not exactly realtime millisceconds. 
        }

        Serial.println(time_ms);

        i++;
        scale.set_scale(calibration_factor); //Adjust to the calibration factor
        Serial.print(scale.get_units(), 1);

        //write to sd card

        myFile.print(time_ms);
        myFile.print(",");
        myFile.println(scale.get_units(), 1);
      }
    }
    else {
        Serial.println("ABORT: bailout");
        RED();
        //use GOTO fn.
        delay(5000);
        goto bailout;
        
    }
    
    bailout: //ABORT at the last second.
    digitalWrite(LED, LOW);
    myFile.close();

    delay(1000);

    tone(buzzer, 2000, 300);
    delay(500);
    tone(buzzer, 3000, 300);
    tone(buzzer, 3500, 300);
    delay(500);
    tone(buzzer, 2500, 300);

    scale.power_down(); // power down loadcell
    #endif
  }

  //buttonState = 1;

  digitalWrite(mos, LOW);
}

//Create a new filename everytime.
boolean loadSDFile()
{
  int i = 0;
  boolean file = false;

  while (!file && i < 1024)
  {
    filename = (String)i + "TB.csv"; //Thrust Bench

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

void GREEN_Idle() {

  //Everything is fine.. signal.
  unsigned long interval = 1000;
  unsigned long mid_val;
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    mid_val = currentMillis - previousMillis;
    digitalWrite(GLED, HIGH);

    tone(7, 2500, 100);

  }
  else {
    digitalWrite(GLED, LOW);
  }
}

void ABORT() {
  Hold = true;
}
