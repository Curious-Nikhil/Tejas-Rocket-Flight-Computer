#include <Wire.h>
#include "i2c.h"
#include "i2c_BMP280.h"

#include <SPI.h>
#include <SD.h>

BMP280 bmp280;
File dataFile;


#define GLED 9
#define RLED 6
static float meters;
float temperature;
float pascal;
int sdcount = 0;
bool err = false;

const int chipSelect = 4;
void setup()
{

  Serial.begin(38400);

  Serial.print("Probe BMP280: ");
  if (bmp280.initialize()) Serial.println("Sensor found");
  else
  {
    Serial.println("Sensor missing");
    err = true;
    while (1) {}
  }


  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    while (1);
  }
  Serial.println("card initialized.");
  

//Calibration Settings - https://www.best-microcontroller-projects.com/bmp280.html#L1080
bmp280.setPressureOversampleRatio(10); //Oversampling Ratio!
bmp280.setTemperatureOversampleRatio(1);
bmp280.setFilterRatio(4);
bmp280.setStandby(0);


// onetime-measure:
bmp280.setEnabled(0);

}

void loop()
{
  dataFile = SD.open("datalog.txt", FILE_WRITE);
  bmp280.awaitMeasurement();

  float temperature;
  bmp280.getTemperature(temperature);

  float pascal;
  bmp280.getPressure(pascal);

  static float meters, metersold;
  //    bmp280.getAltitude(meters);
  bmp280.getAltitude(meters, 101800.00); // forcast  - 71300 default - 101800

  bmp280.triggerMeasurement();


  Serial.print(meters);
  Serial.print(" Pressure: ");
  Serial.print(pascal);
  Serial.print(" Pa; T: ");
  Serial.print(temperature);
  Serial.println(" C");

  dataFile.println(meters);

  delay(10);


}
