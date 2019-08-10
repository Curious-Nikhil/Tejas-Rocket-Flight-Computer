
#include <SPI.h>
#include <SD.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>
#include <Servo.h>
File myFile; //creates a file to be written to
Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085); // sets up the BMP180
int altitude;
int lastAltitude;
int apogee; // Stores max altitude for total height
int groundAltitude; // Stores starting altitude, before launch
int maxHeight;
byte chipSelect = 4; // for sd card reader
int groundTemp;
int apogeeTemp;
int tempChg;
float temperature;
Servo myservo; // Creates servo object
byte yellowLed = 3;
byte redLed = 2;

void setup() {
pinMode(yellowLed, OUTPUT);
pinMode(redLed, OUTPUT);
Serial.begin(9600); // Begins the serial monitor for debugging
pinMode(4, OUTPUT); // for sd card
Serial.print(F("Initializing SD card..."));
  pinMode(10, OUTPUT);

  if (!SD.begin(4)) {
    Serial.println(F("initialization failed!"));
    digitalWrite(redLed, HIGH);
    return;
  }
  Serial.println(F("initialization done."));
  myFile = SD.open("data.txt", FILE_WRITE);
    myFile.println(" ");
    myFile.println("Starting data logging");
    myFile.println("---------------------");
    delay(50);
    myFile.close();
if(!bmp.begin()) // Initializes the BMP180
  {
    while(1);
  }
  myservo.attach(9); // Mounts the servo to pin 9
  myservo.write(20); // Writes starting pos of servo
  sensors_event_t event; // Starts a reading
  bmp.getEvent(&event);
  groundAltitude = bmp.pressureToAltitude(1013,event.pressure); //reads altitude
  bmp.getTemperature(&temperature); //reads temp
  groundTemp = temperature;
  myFile = SD.open("data.txt", FILE_WRITE);
    myFile.print(F("Altitude before Launch was "));
    myFile.print(groundAltitude);
    myFile.println(F(" meters."));
    myFile.print(F("Temperature before launch was "));
    myFile.print(groundTemp);
    myFile.println(F(" Degrees C."));
    myFile.close();
    digitalWrite(yellowLed, HIGH);
    digitalWrite(redLed, LOW);
    
}

void loop() {
  getAltitude(); // Gets current altitude
  if(altitude - lastAltitude <= -1) // Checks for a 1 meter drop in altitude
  {
    Serial.println("                                         TRIGGERED, MUST BE VERIFIED");
    delay(150); // Gravity is 0.102ms per meter, so 150ms is enough to compensate for drag.
    getAltitude; // Gets a new altitude reading
    if(altitude - lastAltitude <= -2) // Checks if new altitude reading is 1 meter less than last measured
    {
      Serial.println("                                         TRIGGERED 2, MUST BE VERIFIED");
      delay(150); // Gravity is 0.102ms per meter, so 150ms is enough to compensate for drag.
      getAltitude; // Gets a new altitude reading
      if(altitude - lastAltitude <= -3) // Checks if new altitude reading is 2 meters less than last measured
      {
        myservo.write(180); // Moves servo to deploy parachute
        Serial.println("                         DEPLOYING");
        bmp.getTemperature(&temperature);
        apogeeTemp = temperature;
        myFile = SD.open("data.txt", FILE_WRITE);
        myFile.println("Parachute deploying!");
        myFile.close();
        apogee = lastAltitude;
        apogee = apogee - 3;
        maxHeight = apogee - groundAltitude;
        tempChg = groundTemp - apogeeTemp;
        Serial.print(F("Parachute has been deployed. Max Height of rocket was "));
        Serial.print(maxHeight);
        Serial.println(F("meters"));
        myFile = SD.open("data.txt", FILE_WRITE);
        myFile.print(F("Max Height of rocket was "));
        myFile.print(maxHeight);
        myFile.println(F("meters."));
        myFile.print(F("The altitude at apogee was "));
        myFile.print(apogee);
        myFile.println(F("meters"));
        myFile.print(F("The temperature at apogee was "));
        myFile.print(apogeeTemp);
        myFile.println(F(" C"));
        myFile.print("There was a ");
        myFile.print(tempChg);
        myFile.print(F(" degree C change from launch to apogee"));
        myFile.close();
        while(1)
        {
        digitalWrite(yellowLed, LOW);
        delay(1000);
        digitalWrite(yellowLed, HIGH);
        delay(1000); 
        } 
      }
      else
      {
        lastAltitude = altitude; // False reading. Sets current altitude to the last measured altitude
        Serial.println("                                             Didnt fall, 3");
      }
    }
    else
    {
      lastAltitude = altitude; // False reading. Sets current altitude to the last measured altitude
      Serial.println("                                                 Didnt fall, 2");
    }
  }
  else
  {
    lastAltitude = altitude; // False reading. Sets current altitude to the last measured altitude
    Serial.println("                                                 Didnt fall");
  }
}

void getAltitude() // Reads the BMP180 and returns an altitude in meters
{
  sensors_event_t event; // Starts a reading
  bmp.getEvent(&event);
  altitude = bmp.pressureToAltitude(1013,event.pressure); // Sets the altitude to int 'altitude'
  Serial.print(altitude); // Prints Altitude to Serial monitor
 Serial.println( "meters");
 }
