#include "HX711.h"
#include <SD.h>

#define mos 6
#define pushbutton 5
#define buzzer 7
#define RLED 9 // Red Light
#define GLED 8

//For Debug:
//#define LOADCELL_CHECK

String command;
int mode = 0;
int buttonState = 0;
int i = 0;
bool BT_FIRE = 0;
bool BT_ABORT = 0;
bool BT_AVAILABLE = 0;
bool Fire_State = 0;
bool COUNTER_STATUS = 0;
bool BS_LOOP = 0;

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
  pinMode(RLED, OUTPUT);
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

  /**
    MODES:
    Mode 0: Invalid Waiting for mode
    Mode 1: Default Launcpad without SD and Loadcell checks
    Mode 2: Test Bench Launchpad - with checks.
  */
  Serial.println(Serial.available());
  Serial.println("Waiting for Serial/BT command");
  Serial.println("Enter Launchpad Mode:");

  digitalWrite(RLED, HIGH);
  while (Serial.available() == 0) {

  }

  digitalWrite(RLED, LOW);
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');

    Serial.println(command);
    if (command.equals("1")) {
      mode = 1;
      Serial.println("M1: DEFAULT LAUNCPAD");
      tone(buzzer, 2500, 2000);
      digitalWrite(GLED, HIGH);
    }
    else if (command.equals("2")) {
      Serial.println("M2: TEST BENCH LAUNCPAD");
      mode = 2;
      tone(buzzer, 2500, 2000);
      digitalWrite(GLED, HIGH);
      delay(1000);
      digitalWrite(GLED, LOW);
      delay(1000);
      digitalWrite(GLED, LOW);
    }
    else {
      Serial.println("Invalid command");
      Serial.println("Reset Launcpad");
      RED();
      while (1);
    }
  }


  if (mode == 1) {
    Serial.println("M1");
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
  }
  if (mode == 2) {
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
    //scale.power_down();
#endif
  }



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

  if (Serial.available()) {
    command = Serial.readStringUntil('\n');

    BT_AVAILABLE = 1;
    Serial.println(command);
    if (command.equals("F1")) {
      BT_FIRE = 1;
      buttonState = 1; //bug
      Serial.println("DEFAULT Launcpad");
    }
    else if (command.equals("A")) {
      BT_ABORT = 1;
      Serial.println("ABORTING");
    }
    else {
      Serial.println("Invalid command");
    }
  }


  digitalWrite(mos, LOW); //Set Mos pin LOW, dont launch by accident!!

  if (COUNTER_STATUS == 0 && BT_FIRE == 0) {
    GREEN_Idle();
  }
  // RESET COUNTER
  //Buttons no longer can ABORT THE LAUNCH - COUNTER_STATUS == 1 && buttonState == 1
  else if (COUNTER_STATUS == 1 && buttonState == 1 && BT_AVAILABLE == 0) {
    Serial.println("HOLD COUNTER");
    Serial.println("RESET COUNTDOWN");
    COUNTER_STATUS = 0;
    counter = 0;
    BS_LOOP = 0;
    // BT_FIRE = 0;
    // BT_ABORT = 0;
    HOLD_IDLE();
    delay(5000);
  }
  else if (buttonState == 1 && COUNTER_STATUS == 1 && BT_ABORT == 1) {
    Serial.println("HOLD COUNTER");
    Serial.println("RESET COUNTDOWN");
    COUNTER_STATUS = 0;
    counter = 0;
    BS_LOOP = 0;
    BT_FIRE = 0;
    BT_ABORT = 0;
    HOLD_IDLE();
    delay(5000);
  }
  else if (COUNTER_STATUS == 1 && buttonState == 0 && Fire_State == 1 || COUNTER_STATUS == 1 && BT_FIRE == 1 && Fire_State == 1) {
    BT_FIRE = 0;
    COUNTER_STATUS = 0;
    POST_LAUNCH();
    Fire_State = 1;
  }



  //1 - OFF, 0 - ON. - using INPUT_PULLUP.
  if (buttonState == 0 && Fire_State == 0 || BT_FIRE == 1 && Fire_State == 0)
  {

    if (COUNTER_STATUS == 0 && BS_LOOP == 0) {
      Serial.println("FIRE BUTTON ENABLED");
      Serial.println("INITIATING LAUNCH PROGRAM! GODSPEED!");
    }

    BS_LOOP = 1; // check for button loop to print above message.


    //Check for file and loadcell error

    if (mode == 2) {
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

    currentMillis = millis();

    if (currentMillis - previousMillis >= (time_interval))  {

      COUNTER_STATUS = 1;
      previousMillis = currentMillis;
      counter++;

      if (counter == 1) {
        Serial.println("Starting a 10 second Timer!");
        Serial.println(10 - counter);
        tone(buzzer, 3500, 500);
        digitalWrite(RLED, HIGH);

        if (mode == 2) {
          myFile = SD.open(filename, FILE_WRITE);

          scale.power_up();
          scale.set_scale(calibration_factor);
          scale.tare();

          //Check for file and loadcell error
#ifdef LOADCELL_CHECK
          if (myFile == 0)
          {
            RED();
            Serial.println("File - ERROR = 0");
            while (1);
          }
          if (scale.read() < 1000)
          {
            RED();
            Serial.println("Loadcel - ERROR = 0");
            while (1);
          }
          Serial.println("Loadcel and SD Mod is set up");
#endif
        }
      }
      //Launch Sequence Starts
      if (counter % 2 == 0)
      {
        //If even
        digitalWrite(RLED, HIGH);
        Serial.println(10 - counter);
        tone(buzzer, 3500, 500);
      }
      if (counter % 2 == 1)
      {
        digitalWrite(RLED, LOW);
        Serial.println(10 - counter);
        tone(buzzer, 3000, 500);
      }
    }

    if (counter >= 10 && Fire_State == 0 || counter >= 10 && Fire_State == 0 && BT_FIRE == 1)
    {
      Serial.println("LAUNCH OFF");
      tone(buzzer, 4500, 2000);
      digitalWrite(RLED, HIGH);
      digitalWrite(mos, HIGH);

      previousMillis = 0;

      if (mode == 2) {
        while (i < 100)

        {
          scale.power_up();
          scale.set_scale(calibration_factor);
          scale.tare();
          Serial.println(scale.read());
          delay(1000);

          digitalWrite(mos, HIGH); // too dangerous.

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
          myFile.println(scale.get_units(), 1);
          Serial.print(time_ms);
          Serial.print(",");
          Serial.println(scale.get_units(), 1);
          i++;
        }
      }

      if (mode == 1 && COUNTER_STATUS == 1) {
        while ( i < 100) {
          
          Serial.print(millis());
          Serial.print(",");
          Serial.println(scale.get_units(), 1);
        }
        while ( i < 100) {
          Serial.print(millis());
          Serial.print(",");
          Serial.println(scale.get_units(), 1);

          digitalWrite(mos, HIGH);

        }
        delay(5000);
        Serial.println("M1 - LAUNCH PROGRAM ENDS");
      }

      Fire_State = 1;

      if (mode == 2) {
        myFile.close();
        scale.power_down();
        Serial.println("M2 - LAUNCH PROGRAM ENDS");
      }


      tone(buzzer, 2000, 300);
      delay(500);
      tone(buzzer, 3000, 300);
      tone(buzzer, 3500, 300);
      delay(500);
      tone(buzzer, 2500, 300);
    }

    buttonState = digitalRead(pushbutton); //check if Fire Button is still "ON"
  }

  digitalWrite(RLED, LOW);
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
  digitalWrite(RLED, HIGH);
  tone(7, 2500, 100);
  delay(200);
  digitalWrite(RLED, LOW);

  tone(7, 2500, 100);
  delay(200);
  digitalWrite(RLED, HIGH);

  tone(7, 2000, 100);
  delay(500);
  tone(7, 2000, 100);
}

void GREEN_Idle() {
  //Everything is fine.. signal.
  unsigned long interval = 1000;
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    digitalWrite(GLED, HIGH);

    tone(7, 2500, 100);

  }
  digitalWrite(GLED, LOW);
  digitalWrite(RLED, LOW);
}

void HOLD_IDLE() {
  //Counter is HOLD> Signal
  tone(7, 3200, 2000);
  delay(1000);
  for (int i = 1; i < 20; i++) {
    digitalWrite(RLED, HIGH);
    digitalWrite(GLED, LOW);
    tone(7, (i * 10) + 2400, 25);
    delay(50);
    digitalWrite(RLED, LOW);
    digitalWrite(GLED, HIGH);
    delay(50);
    tone(7, 2500, 25);
  }

  digitalWrite(GLED, LOW);
}

void POST_LAUNCH() {

  unsigned long interval = 3000;
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    Serial.println("RELEASE FIRE BUTTON!");

    tone(7, 2500, 100);

  }

  digitalWrite(RLED, HIGH);

}
