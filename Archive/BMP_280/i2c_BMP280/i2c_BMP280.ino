#include <Wire.h>
#include "MPU6050.h"
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
int sdcount = 0;
bool err = false;
String filename; 
File dataFile;

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;

void setup()
{
  delay(5000);
    Serial.begin(38400);

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
    bmp280.triggerMeasurement();

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
    

    //Print Header Files
    dataFile = SD.open(filename, FILE_WRITE);

    dataFile.print("Time");
    dataFile.print(",");    
    dataFile.print("Alt");
    dataFile.print(",");
    dataFile.print("KMAlt");
    dataFile.print(",");
    dataFile.print("Pascal");
    dataFile.print(",");
    dataFile.print("ax");   
    dataFile.print(",");
    dataFile.print("ay");
    dataFile.print(",");
    dataFile.println("az");

    dataFile.close();

    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

}

void loop()
{
    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();

    get_alt();

    accelgyro.getAcceleration(&ax, &ay, &az);

    float est_alt = pressureKalmanFilter.updateEstimate(meters);
    
    //Data Logging
      dataFile = SD.open(filename, FILE_WRITE);

      Serial.println(FreeRam());

      if (dataFile) {
      //Serial Prints
      Serial.print(meters);
      Serial.print(",");
      Serial.print(pascal);
      Serial.print(",");
      Serial.println(est_alt);
      
      //Writing in SD Card!
      dataFile.print(millis());
      dataFile.print(",");
      dataFile.print(meters);   
      dataFile.print(",");
      dataFile.print(est_alt);
      dataFile.print(",");
      dataFile.print(pascal);
      dataFile.print(",");
      dataFile.print(ax);   
      dataFile.print(",");
      dataFile.print(ay);
      dataFile.print(",");
      dataFile.println(az);
      dataFile.close();
      sdcount++;

      if (sdcount > 100) {
        dataFile.flush();
        sdcount = 0;
      }
      delay(10);

      } else {
      Serial.println("error Opening the txt file");
      err = true;
      }

}

boolean loadSDFile() {
  int i = 0;
  boolean file = false;

  while(!file && i < 1024) {
    filename = (String)i + "FLT.csv";

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

void get_alt() {
  bmp280.getAltitude(meters);
  bmp280.getPressure(pascal);
  float est_alt = pressureKalmanFilter.updateEstimate(meters);

  return est_alt;
}

// void writeSD(float meters, float pascal, float est_alt) {

//   dataFile = SD.open(filename, FILE_WRITE);

//   Serial.println(FreeRam());

//   if (dataFile) {
//   //Serial Prints
//   Serial.print(meters);
//   Serial.print(",");
//   Serial.print(pascal);
//   Serial.print(",");
//   Serial.println(est_alt);
  
//   //Writing in SD Card!
//   dataFile.print(meters);   
//   dataFile.print(",");
//   dataFile.print(est_alt);
//   dataFile.print(",");
//   dataFile.println(pascal);
//   dataFile.close();

//   delay(10);

//   } else {
//   Serial.println("error Opening the txt file");
//   err = true;
//   }
// }
