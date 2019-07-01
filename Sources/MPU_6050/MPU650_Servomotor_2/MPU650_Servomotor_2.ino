#include <Wire.h>
#include <MPU6050.h>
#include <Servo.h>

Servo sg90;

int servo1_pin = 2;

MPU6050 sensor ;

int16_t ax, ay, az; // variables for acc
int16_t gx, gy, gz; //vars for gyroscope


void setup() {
  sg90.attach(servo1_pin);

  Wire.begin();

  Serial.begin(9600);
}

void loop() {
  Serial.println(sensor.testConnection() ? "Yes" : "NO");
  
}
