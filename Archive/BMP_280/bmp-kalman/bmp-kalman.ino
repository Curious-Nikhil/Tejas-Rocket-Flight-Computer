#include <Wire.h>
#include "i2c_BMP280.h"
#include "SimpleKalmanFilter.h"

BMP280 bmp;
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);


float meters;
float kalmanmeters;
float temperature;
float pascal;

void setup()
{
  Serial.begin(115200);


  Serial.print("Probe bmp: ");
  if (bmp.initialize()) Serial.println("Sensor found");
  else
  {
    Serial.println("Sensor missing");
    while (1) {}
  }

  bmp.setPressureOversampleRatio(10); //Oversampling Ratio!
  bmp.setTemperatureOversampleRatio(1);
  bmp.setFilterRatio(4);
  bmp.setStandby(0);
  
  // onetime-measure:
  bmp.setEnabled(0);
  bmp.triggerMeasurement();
}

void loop()
{
  bmp.awaitMeasurement();
  bmp.getTemperature(temperature);
  bmp.getPressure(pascal);
  bmp.getAltitude(meters);
  bmp.triggerMeasurement();

  float est_alt = pressureKalmanFilter.updateEstimate(meters);
  
  Serial.print(" m; Height: ");
  Serial.print(meters);
  Serial.print(" m; SKalmanH: ");
  Serial.println(est_alt);
  //  Serial.print(" Pressure: ");
  //  Serial.print(pascal);
  //  Serial.print(" Pa; T: ");
  //  Serial.print(temperature);
  //  Serial.println(" C");
}
