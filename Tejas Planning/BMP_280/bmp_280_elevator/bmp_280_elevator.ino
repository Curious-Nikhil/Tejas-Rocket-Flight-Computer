#include <Wire.h>
#include "i2c.h"

#include "i2c_BMP280.h"

BMP280 bmp280;

void setup()
{
    Serial.begin(115200);
    Serial.println("BMP280 elevator");
    Serial.print("Probe BMP280: ");
    if (bmp280.initialize()) Serial.println("Sensor found");
    else
    {
        Serial.println("Sensor missing");
        while (1) {}
    }

    // Setup for elevator floor change measurement.
    bmp280.setPressureOversampleRatio(2);
    bmp280.setTemperatureOversampleRatio(1);
    bmp280.setFilterRatio(0);
    bmp280.setStandby(0);

    // onetime-measure:
    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();
}

float get_altitude() {
   float meters;
   bmp280.getTemperature(meters);  // throw away - needed for alt.
   bmp280.getPressure(meters);     // throw away - needed for alt.
   bmp280.getAltitude(meters);
   return meters;
}

void loop()
{
    float distance,velocity;
    static uint32_t time_now, time_was=millis(),dt;
    static float alt_was=get_altitude();
    static uint32_t sample_time_was=millis();
    static uint32_t readings=0;
    static float values=0,avg_alt=0;
    static float meters;

    if ( (millis()-sample_time_was) >2) { // read at a specific rate.

       if(!bmp280.awaitMeasurement()) Serial.println("MEASURE FAILED");

       meters = get_altitude();
       readings++;
       values += meters;

       bmp280.triggerMeasurement();

       sample_time_was = millis();
    }

//     // Serial plotter
//     Serial.print(meters);
//     Serial.print(" ");
//     Serial.println(metersoldn);

    time_now = millis();
    dt = time_now - time_was;
    if (dt>=1000) {

       avg_alt = values/readings;
       distance = avg_alt - alt_was;
       velocity = (distance/((float)dt/1000.0));
       alt_was  = avg_alt;
       time_was = time_now;

//    Serial.print("AVG: ");
//    Serial.print(avg_alt);
//    Serial.print(" m/s ; dist: ");
//    Serial.print(distance);

    if (fabs(velocity)>0.2) {
       Serial.print("Velocity: ");
       Serial.print(velocity);
       Serial.print(" m/s ;");
    }
    Serial.print("debug: ");
    Serial.println(readings);


    values=0;
    readings=0;
    }

}


