#include "HX711.h"
#include <SD.h>

#define mos 2
#define pushbutton 5
#define buzzer 7
#define LED 9 // Red Light
#define GLED 8

//For Debug:
//#define LOADCELL_CHECK

int buttonState = 0;
int i = 0;
bool Fire_State = 0;
bool COUNTER_STATUS = 0;

//LOADCELL Stuff here
#define DOUT 2
#define CLK 3

HX711 scale;
float calibration_factor = 239;
float weight = 0;

//Timer
int time_ms;
int counter = 0;
unsigned long previousMillis;
unsigned long startMillis;
unsigned long currentMillis;
unsigned long time_interval = 1000;
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
  pinMode(GLED, OUTPUT);
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
    while (1);
  }
}

void loop()
{
  buttonState = digitalRead(pushbutton);
  digitalWrite(mos, LOW); //Set Mos pin LOW, dont launch by accident!!

  if (COUNTER_STATUS == 0) {
    GREEN_Idle();
    Serial.println(buttonState);

  }
  // RESET COUNTER
  else if (COUNTER_STATUS == 1 && buttonState == 1) {
    Serial.println("HOLD COUNTER");
    Serial.println("RESET COUNTDOWN");
    COUNTER_STATUS = 0; 
    counter = 0;
    HOLD_IDLE();
    delay(5000);
  }
  else if (COUNTER_STATUS == 1 && buttonState == 0 && Fire_State == 1) {
  Serial.println("RELEASE FIRE BUTTON!");
  POST_LAUNCH();
  delay(5000);
  digitalWrite(LED, HIGH);
  while(1);
  }
  



  //1 - OFF, 0 - ON. - using INPUT_PULLUP.
  if (buttonState == 0 && Fire_State == 0)
  {
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
    currentMillis = millis();

    if (currentMillis - previousMillis >= (time_interval))  {

      Serial.println(currentMillis - previousMillis);
      COUNTER_STATUS = 1;
      previousMillis = currentMillis;
      counter++;

      if (counter == 1) {
        Serial.println("Starting a 10 second Timer!");
        Serial.println(10 - counter);
        tone(buzzer, 3500, 500);
        digitalWrite(LED, HIGH);

        //check for systems once again.
        myFile = SD.open(filename, FILE_WRITE);

        scale.power_up();
        scale.set_scale(calibration_factor); 
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
      }
      //Launch Sequence Starts
      if (counter % 2 == 0)
      {
        //If even
        digitalWrite(LED, HIGH);
        Serial.println(10 - counter);
        tone(buzzer, 3500, 500);
      }
      if (counter % 2 == 1)
      {
        digitalWrite(LED, LOW);
        Serial.println(10 - counter);
        tone(buzzer, 3000, 500);
      }
    }

    if (counter >= 10 && Fire_State == 0)
    {
      Serial.println("LAUNCH OFF");
      tone(buzzer, 4500, 2000);
      digitalWrite(LED, HIGH);
      digitalWrite(mos, HIGH);

      previousMillis = 0;
      while (i < 100)
      {
        digitalWrite(mos, HIGH); 

        Serial.println("While Loop");
        //Time Instance
        currentMillis = millis();
        
        if (currentMillis - previousMillis >= period) {
          previousMillis = currentMillis;

          time_ms++;  //not exactly realtime millisceconds.
        }

        Serial.println(time_ms);
        //Serial.println(scale.get_units(), 1);

        //write to sd card

        myFile.print(time_ms);
        myFile.print(",");
        //myFile.println(scale.get_units(), 1);

        i++;   
      }

      Fire_State = 1;
      myFile.close();
      scale.power_down(); // power down loadcell

      tone(buzzer, 2000, 300);
      delay(500);
      tone(buzzer, 3000, 300);
      tone(buzzer, 3500, 300);
      delay(500);
      tone(buzzer, 2500, 300);

      Serial.println("Launch Program ENDS");
    }

    buttonState = digitalRead(pushbutton); //check if Fire Button is still "ON"
  }

  digitalWrite(LED, LOW);
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

void HOLD_IDLE() {
  //Counter is HOLD> Signal
  tone(7, 3200, 2000);
  delay(1000);
  for (int i = 1; i < 20; i++) {
    digitalWrite(LED, HIGH);
    digitalWrite(GLED, LOW);
    tone(7, (i * 10) + 2400, 25);
    delay(50);
    digitalWrite(LED, LOW);
    digitalWrite(GLED, HIGH);
    delay(50);
    tone(7, 2500, 25);
  }

  digitalWrite(GLED, LOW);
}

void POST_LAUNCH() {
  //Counter is HOLD> Signal
  tone(7, 3200, 2000);
  delay(1000);
  for (int i = 1; i < 10; i++) {
    digitalWrite(LED, HIGH);
    digitalWrite(GLED, LOW);
    tone(7, (i * 10) + 2400, 25);
    delay(50);
    digitalWrite(LED, LOW);
    digitalWrite(GLED, HIGH);
    delay(50);
    tone(7, 2500, 25);
  }

  for (int i = 10; i < 1; i++) {
  digitalWrite(LED, HIGH);
  digitalWrite(GLED, LOW);
  tone(7, (i * 10) + 2400, 25);
  delay(50);
  digitalWrite(LED, LOW);
  digitalWrite(GLED, HIGH);
  delay(50);
  tone(7, 2500, 25);
  }

  digitalWrite(GLED, LOW);
}