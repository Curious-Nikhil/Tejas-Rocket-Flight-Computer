#include <Wire.h>
#include "i2c.h"
#include "i2c_BMP280.h"
#include "SimpleKalmanFilter.h"

#include <SPI.h>
#include<SD.h>

BMP280 bmp280;
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

#define GLED 9
#define RLED 6
static float meters;
float temperature;
float pascal;

bool err = false;
String filename; 
File dataFile;

void setup()
{
    Serial.begin(115200);

    Serial.print("Probe BMP280: ");
    if (bmp280.initialize()) Serial.println("Sensor found");
    else
    {
        Serial.println("Sensor missing");
        err = true;
        while (1) {}
    }

    //Calibration Settings - https://www.best-microcontroller-projects.com/bmp280.html#L1080
    bmp280.setPressureOversampleRatio(10); //Oversampling Ratio!
    bmp280.setTemperatureOversampleRatio(1);
    bmp280.setFilterRatio(4);
    bmp280.setStandby(0);


    // onetime-measure:
    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();\

    // see if the card is present and can be initialized:
    if (!SD.begin(4)) {
      Serial.println("Card failed, or not present");
      err = true;
      // don't do anything more:
      while (1);
    }
    Serial.println("card initialized.");

    //Create a file with new name
    if (!loadSDFile()) {
      Serial.println("Failed to create file");
      err = true;
    }
    else {
      Serial.println("File name created!");
    }

    pinMode(GLED, OUTPUT);
    pinMode(RLED, OUTPUT);
    
    //Error Conditions/Check
    if (err) {
      digitalWrite(RLED, HIGH);
    }
    else {
      digitalWrite(GLED, HIGH);
      }
      
    //Print Header Files
    dataFile = SD.open(filename, FILE_WRITE);
    dataFile.print("Height");
    dataFile.print(",");
    dataFile.print("Kalman Est Alt");
    dataFile.print(",");
    dataFile.println("Pascal");
    dataFile.close();
}

void loop()
{
    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();

    bmp280.getTemperature(temperature);  // throw away - needed for alt.
    bmp280.getPressure(pascal);     // throw away - needed for alt.
    bmp280.getAltitude(meters);

    float est_alt = pressureKalmanFilter.updateEstimate(meters);
    
    //Data Logging
    writeSD(meters, pascal, est_alt);

    if (err) {
      digitalWrite(RLED, HIGH);
    }
    else {
      digitalWrite(GLED, HIGH);
      }
      
}

boolean loadSDFile() {
  int i = 0;
  boolean file = false;

  while(!file && i < 1024) {
    filename = (String)i + "datalog.txt";

    if (!SD.exists(filename)) {
      dataFile = SD.open(filename, FILE_WRITE);
      delay(10);
      dataFile.close();
      file = true;
    }
    //else file = false
    i++;
  }

  return file;
}

void writeSD(float meters, float pascal, float est_alt) {

  dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
  //Serial Prints
  Serial.print(meters);
  Serial.print(",");
  Serial.print(pascal);
  Serial.print(",");
  Serial.println(est_alt);
  
  //Writing in SD Card!
  dataFile.print(meters);   
  dataFile.print(",");
  dataFile.print(est_alt);
  dataFile.print(",");
  dataFile.println(pascal);
  dataFile.close();

  delay(500);

  } else {
  Serial.println("error Opening the txt file");
  err = true;
  }
}
