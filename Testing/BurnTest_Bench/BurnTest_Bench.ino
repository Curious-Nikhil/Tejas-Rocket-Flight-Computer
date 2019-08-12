#include "HX711.h"
#include <SD.h>

#define mos 6
#define toggleButton 5
#define buzzer 7
#define RLED 9 // Red Light
#define GLED 8 //Green Light

//For Debug:
#define LOADCELL_CHECK

String command;
int mode = 0;
int buttonState = 0;
int i = 0;
int timer = 0;
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
  pinMode(toggleButton, INPUT_PULLUP);

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
  Serial.println(F("Waiting for Serial/BT command"));
  Serial.println(F("Enter Launchpad Mode:"));

  digitalWrite(RLED, HIGH);
  while (Serial.available() == 0) {

  }

  digitalWrite(RLED, LOW);
  if (Serial.available()) {
    command = Serial.readStringUntil('\n');

    Serial.println(command);
    if (command.equals("1")) {
      mode = 1;
      Serial.println(F("M1"));
      tone(buzzer, 2500, 2000);
      digitalWrite(GLED, HIGH);
    }
    else if (command.equals("2")) {
      Serial.println(F("M2"));
      mode = 2;
      tone(buzzer, 2500, 2000);
      digitalWrite(GLED, HIGH);
      delay(1000);
      digitalWrite(GLED, LOW);
      delay(1000);
      digitalWrite(GLED, LOW);
    }
    else {
      Serial.println(F("x com"));
      Serial.println(F("Reset Launcpad"));
      RED();
      while (1);
    }
  }


  if (mode == 2) {
    //SD CARD initialization
    initializeSD();

    //Check if Loadcel is working.
    scale.begin(DOUT, CLK);

    scale.set_scale();
    scale.tare();
    long reading;

      #ifdef LOADCELL_CHECK
    for (int i=0; i<10; i++) {
      Serial.println(scale.read());
      // && scale.read() > 1000 && scale.read() < 2000
      if (scale.wait_ready_timeout(1000) == 0 || scale.read() <1200)
      {
        Serial.println(F("HX711 not Found"));
        Serial.println(scale.read());

        RED();
        while (1);
      }
      Serial.println(scale.read());

    }

    scale.power_down();
      #endif

   }



  //check if Fire Switch is enabled?
  buttonState = digitalRead(toggleButton);
  if (buttonState == 0) {
    //Error state, need to switch it off
    Serial.println(F("Fire Enable: Error"));
    RED();
    while (1);
  }
}

void loop()
{
  buttonState = digitalRead(toggleButton);


  if (Serial.available()) {
    BT_AVAILABLE = 1;

    command = Serial.readStringUntil('\n');
    command.trim();
    Serial.println(command);

    if (command.equals("F1")) {
      tone(buzzer, 2500, 1500);
      digitalWrite(GLED, HIGH);
      Serial.println(F("AUTHORIZE FIRE KEY"));

      while (Serial.available() == 0) {}

      command = Serial.readStringUntil('\n');
      command.trim();
      Serial.println(command);
      delay(50);
      if (command.equals("F2")) {
        BT_FIRE = 1;
        buttonState = 1; //bug
        Serial.println(command);
        Serial.println(F("LAUNCH AUTHORIZED!"));
        digitalWrite(GLED, LOW);

      }
      else {
        Serial.println(command);
        Serial.println(F("LAUNCH ABORTED - FAILED AUTHORIZED"));
      }

    }
    else if (command.equals("A")) {
      BT_ABORT = 1;
      Serial.println(F("ABORTING"));
    }
    else {
      Serial.println(F("X Com"));
    }
  }


  digitalWrite(mos, LOW); //Set Mos pin LOW, dont launch by accident!!

  if (COUNTER_STATUS == 0 && BT_FIRE == 0) {
    GREEN_IDLE();
  }
  // ABORT PROGRAM - When usin a ToogleSwitch
  else if (COUNTER_STATUS == 1 && buttonState == 1 && BT_AVAILABLE == 0) {
    Serial.println(F("RESET COUNTDOWN"));
    COUNTER_STATUS = 0;
    counter = 0;
    BS_LOOP = 0;
    // BT_FIRE = 0;
    // BT_ABORT = 0;
    HOLD_IDLE();
    delay(5000);
  }
  // ABORT PROGRAM - When using Bluetooth
  else if (buttonState == 1 && COUNTER_STATUS == 1 && BT_ABORT == 1) {
    Serial.println(F("RESET COUNTDOWN"));
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

    digitalWrite(RLED, LOW);
    delay(5000);
  }



  //1 - OFF, 0 - ON. - using INPUT_PULLUP.
  if (buttonState == 0 && Fire_State == 0 || BT_FIRE == 1 && Fire_State == 0)
  {

    if (COUNTER_STATUS == 0 && BS_LOOP == 0) {
      Serial.println(F("FIRE BUTTON ENABLED"));
      Serial.println(F("INITIATING LAUNCH PROGRAM. GODSPEED!"));
    }

    BS_LOOP = 1; // check for button loop to print above message.


    //Check for file and loadcell error

    if (mode == 2 && COUNTER_STATUS == 0)  {

      //CHECK COMPONENTS PASS 2
      myFile = SD.open(filename, O_READ);

      scale.power_up();
      scale.set_scale(calibration_factor);
      scale.tare();

      #ifdef LOADCELL_CHECK
      if (myFile == 0)
      {
        RED();
        Serial.println(F("File 0"));
        while (1);
      }
      if (scale.read() < 1000)
      {
        RED();
        Serial.println(F("Load 0"));
        while (1);
      }
      Serial.println(F("Mods Set Up"));
      myFile.close();
      #endif
    }

    currentMillis = millis();

    if (currentMillis - previousMillis >= (time_interval))  {

      COUNTER_STATUS = 1;
      previousMillis = currentMillis;
      counter++;

      if (counter == 1) {
        Serial.println(F("Starting a 10 second Timer!"));
        Serial.println(10 - counter);
        tone(buzzer, 3500, 500);
        digitalWrite(RLED, HIGH);
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

      if (mode == 2 && COUNTER_STATUS == 1) {
        myFile = SD.open(filename, FILE_WRITE);
        digitalWrite(mos, HIGH);
        while (i < 200) //approx 9 seconds
        {
          currentMillis = millis();

          // if (currentMillis - previousMillis >= 1) {
          //   previousMillis = currentMillis;

          //   timer++;
          // }
          //Serial.println(scale.get_units(), 1);
          //write to sd card
          myFile.print(currentMillis);
          myFile.print(",");
          myFile.println(scale.get_units(), 1);

          i++;
        }
        myFile.flush();
      }

      if (mode == 1 && COUNTER_STATUS == 1) {
        digitalWrite(mos, HIGH);
        delay(10000);
        Serial.println(F("M1 - LAUNCH PROGRAM ENDS"));
      }

      Fire_State = 1;

      if (mode == 2) {
        myFile.close();
        scale.power_down();
        Serial.println(F("M2 - LAUNCH PROGRAM ENDS"));
      }


      tone(buzzer, 2000, 300);
      delay(500);
      tone(buzzer, 3000, 300);
      tone(buzzer, 3500, 300);
      delay(500);
      tone(buzzer, 2500, 300);
    }

    buttonState = digitalRead(toggleButton); //check if Fire Button is still "ON"
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

    myFile.print("Time");
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

void GREEN_IDLE() {
  //Everything is fine.. signal.
  unsigned long interval = 1000;

  
  currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    digitalWrite(GLED, HIGH);

    tone(7, 2500, 100);

  //  cycle++;s
  }

  // if (cycle%5 == 0) {
  //   Serial.println(F("Standby.."));
  // }

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
