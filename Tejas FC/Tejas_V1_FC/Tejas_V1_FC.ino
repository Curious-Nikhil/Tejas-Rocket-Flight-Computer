// =============================
// == PHASE ZERO & ONE Devs ====
//==============================

/*Tejas V1 - @Nikhil Mishra
   https://curious-nikhil.github.io/
   Tejas is a rocket computer
*/
// =========================================
// ==         CONFIGURATION              ==
//==========================================


//#define PID_MOD
//#define SERVO_MOD
//#define DMP_MOD
#define SERIAL_DEBUG
//#define MPU_CALBIRATION
// =============================
// == Include and Define Vars ==
//==============================

//Header Files
#include <I2Cdev.h>
#include "MPU6050.h"
#include <Wire.h>
#include "i2c_BMP280.h"
#include "SimpleKalmanFilter.h"
#include <SD.h>


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define RLED 6// Green LED
#define GLED 7// Green LED
#define buzzer 8
#define pyroPin 9
// =============================================
// ===          MISC Global Vars             ===
// =============================================

//Timers Vars
unsigned long previousMillis = 0;


bool launch = 0;
bool pyro = false;
bool pyroFired = false;
bool landed = false;
bool ABORT = false;

int lastAlt = 0;
unsigned long launchTime = 0;
unsigned long ApoTime = 0;
unsigned long landTime = 0;
// =============================================
// ===              MPU Vars                 ===
// =============================================

MPU6050 accelgyro;

int16_t ax, ay, az;
int16_t gx, gy, gz;


//INTERRUPT DETECTION ROUTINE
// volatile bool mpuInterrupt = false;// indicates whether MPU interrupt pin has gone high
// void dmpDataReady() {
//   mpuInterrupt = true;
// }

// =============================================
// ===              BAROMETER                ===
// =============================================
BMP280 bmp280;
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);

static float alt;
float est_alt;
float temperature;
float pascal;
float base_alt;
// =============================================
// ===              SD CARD                  ===
// =============================================
String filename;
File myFile;
int sd_count = 0; 
bool FL = false;



// ================================================================
// ===               THE SETUP FUNCTION                       ===
// ================================================================
void setup() {

  Serial.begin(38400);

  //Startup Sound
  // tone(buzzer, 2500, 300);
  // delay(1000);
  // tone(buzzer, 2000, 300);
  // delay(500);
  // tone(buzzer, 3000, 300);
  // tone(buzzer, 3500, 300);
  // delay(500);
  // tone(buzzer, 2500, 300);
  // delay(1000);
  delay(2000);

  // configure LED for output
  //pinMode(LED_PIN, OUTPUT);
  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);
  pinMode(pyroPin, OUTPUT);

  Serial.println(FreeRam()); //set a threshold for that.

  //PASS 1 -- confirm the size.
  if (FreeRam() < 275) {
    Serial.println(F("RAM-0"));
    RED();
    while(1);
  }

  // PASS 2: Initialize SD Module
  initializeSD();

  //PASS 3: Initialize Gyroscope and Servo
    //needs calibration method also.
  initializeMPU();

  //PASS 3: Initialize Baromter
  initializeBMP();

  //PASS 4: Pyro Check

  //Get baseline alt
  // float sum = 0;
  // delay(5000);
  // for (int i=0; i < 30; i++) {

  //   delay(100);

  //   bmp280.getAltitude(base_alt);
  //   sum += base_alt;
  //   Serial.println(base_alt);
  // }

  // base_alt = sum/30.0;
  // Serial.print(F("BASEH: "));
  // Serial.println(base_alt);

  // delay(1000);

}

// ================================================================
// ===                    MAIN LOOP                            ===
// ================================================================
void loop() {
  //Disable Pyros
  digitalWrite(pyroPin, LOW);
  
  if(launch == false && pyro == false && landed == false) {
    //GREEN();
  }
  
  /*************
   * Detect launch
   * Enable ABORT - for extreame tilt.
   * Detect Apogee
   * * Fire Pyros
   * Flight Log
   */
  get_alt();
  // Serial.print(F("BASEH: "));
  // Serial.println(base_alt);
  // Serial.print(F("Est_Alt: "));
  // Serial.println(est_alt);

  motion();

  if (launch == 0) {
    Serial.println(ay);
  }

  if (ay > 10000 || launch == true && landed == false) {
    if (launch == 0) {
      Serial.println(F("LAUNCH! ! !"));
      launchTime = millis();

      myFile = SD.open(filename, FILE_WRITE);
      Serial.println(filename + myFile);
      delay(3000);
    }
    Serial.println("LAUNCH");
    Serial.println(launchTime);
    Serial.println("bool: " + launch);
    
    launch = 1;
    //Open File here.
    //ABORT PROGRAM!
    // if (angle > 30) {
    //   //eject pyros
    //   ABORT = true;
    // }
    
    //APOGEE DETECTION PROGRAM
    get_alt();

    if (est_alt - lastAlt <= -1 && pyro == false && launch == true && pyroFired == false) {
      //check for a meter drop
      //Store time of Apogee Trigger 1
      delay(150);
      get_alt();
      Serial.println(F("P1"));

      if(est_alt - lastAlt <= -2 && pyro == false && launch == true && pyroFired == false) {
        //check for 2 meter drop
        //Store time of Apogee Trigger 1

        delay(150);
        get_alt();
        Serial.println(F("P2"));
        if(est_alt - lastAlt <= -3 && pyro == false && launch == true && pyroFired == false) {
          //PASS 3
          //Store time of Apogee Pyro Fire
          //Fire Pyros!
          pyro = true;
          Serial.println(F("P3"));
        }else {
          lastAlt = est_alt;
        }
      }else{
        lastAlt = est_alt;
      }
    }
    else{
      lastAlt = est_alt;
    }
  } 

  if (pyro == true && launch == true && pyroFired == false) {
    Serial.println(F("pyro"));
    digitalWrite(RLED, HIGH);

    ApoTime = millis();
    delay(1000);
    pyroFired = true;

  }

  if (ay <10000 && launch == true && pyro == true) {
    //Land PROGRAM

    if(landed =! true) {
      //Store Landing time ONCE!
      landTime = millis();
    }
    landed = true;
  }

  //Flight Logs

  if (launch == true) {
    //Flight Logs
    // if (FL = false) {
    //   myFile = SD.open(filename, FILE_WRITE);
    // }
    
    myFile = SD.open(filename, FILE_WRITE);

    writeSD(pascal, alt, est_alt, ax, ay, az, gx, gy, gz, launchTime, ApoTime, landTime);
    sd_count++;

    if (sd_count > 100) {
      myFile.flush();
      sd_count = 0;
    }

    if (landed == true) {
      myFile.close();
    }
    
    FL = true;
  }
}//voidloop end


// ================================================================
// ===               SD CARD Begin                       ===
// ================================================================

void initializeSD() {

  Serial.print(F("InitSD"));

  if (!SD.begin(4)) {
    Serial.println(F("SDFAIL"));
    RED();
    while (1);
  }
  Serial.println(F("SDinit"));

  //Create a file with new name
  if (!loadSDFile()) {
    Serial.println(F("F-0"));
    while (1);
    RED();
  }
  else {
    Serial.println(F("F-1"));
    RED();
  }

  Serial.println(filename);

  myFile = SD.open(filename, FILE_WRITE);
  Serial.println(myFile);
  if (myFile) {
    //Print Header Files  - - alt, pascal, est_alt, mpuPitch, mpuRoll, mpuYaw, OutputX, OutputY

    myFile.print("Time");
    myFile.print(",");
    myFile.print("Pascal");
    myFile.print(",");
    myFile.print("KH");
    myFile.print(",");
    myFile.print("ax");
    myFile.print(",");
    myFile.print("ay");
    myFile.print(",");
    myFile.print("az");
    myFile.print(",");
    myFile.print("gx");
    myFile.print(",");
    myFile.print("gy");
    myFile.print(",");
    myFile.print("gz");
    myFile.print(",");
    myFile.print("launch");
    myFile.print(",");
    myFile.print("Apogee");
    myFile.print(",");
    myFile.println("landed");
 

    myFile.close();
    Serial.println(F("Fin-1"));

  } else {
    Serial.print(F("Fin-0"));
    RED();
    while (1);
  }
}

// ================================================================
// ===                         BAROMETER                       ===
// ================================================================

void initializeBMP() {

  Serial.print(F("InintBMP"));
  if (bmp280.initialize()) Serial.println(F("BMP1")); //sensor found
  else
  {
    Serial.println(F("BMP0")); //Sensor not found
    RED();
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

  Serial.println(F("BMPInit1"));
}


// =========================================================
// ===                          MPU init                      ===
// ==========================================================

void initializeMPU() {
// initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    #ifdef MPU_CALBIRATION 
    Serial.println("Updating internal sensor offsets...");
    // -76	-2359	1688	0	0	0
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    // accelgyro.setXGyroOffset(220);
    // accelgyro.setYGyroOffset(76);
    // accelgyro.setZGyroOffset(-85);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
    #endif
}

void motion() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
}
// ================================================================
// ===           ALTIMETER - BAROMETER & KALMAN FILTER          ===
// ================================================================

// void altimeter() {

//   bmp280.setEnabled(0);
//   bmp280.triggerMeasurement();

//   bmp280.getTemperature(temperature);  // throw away - needed for alt.
//   bmp280.getPressure(pascal);     // throw away - needed for alt.
//   bmp280.getAltitude(alt);

//   float est_alt = pressureKalmanFilter.updateEstimate(alt);

//   Serial.print(alt);
//   Serial.print(",");
//   Serial.print(pascal);
//   Serial.print(",");
//   Serial.println(est_alt);
// }


// ================================================================
// ===           SD CARD WRITE AND STUFF                        ===
// ================================================================
//Create a new filename everytime.
boolean loadSDFile() {
  int i = 0;
  boolean file = false;

  while (!file && i < 1024) {
    filename = (String)i + "FL.txt";

    if (!SD.exists(filename)) {
      myFile = SD.open(filename, FILE_WRITE);
      delay(10);
      myFile.close();
      file = true;
    }
    i++;
  }

  return file;
}
// alt, pascal, est_alt
void writeSD(float pascal, float alt, float est_alt, 
  int16_t ax, int16_t ay, int16_t az, 
  int16_t gx, int16_t gy, int16_t gz, 
  unsigned long launchTime, unsigned long ApoTime, 
  unsigned long landTime) {

  myFile = SD.open(filename, FILE_WRITE);

  if (myFile) {
    //Writing in SD Card!
    myFile.print(millis());
    myFile.print(",");
    myFile.print(pascal);
    myFile.print(",");
    myFile.print(alt);
    myFile.print(",");
    myFile.print(est_alt);
    myFile.print(",");
    myFile.print(ax);
    myFile.print(",");
    myFile.print(ay);
    myFile.print(",");
    myFile.print(az);
    myFile.print(",");
    myFile.print(gx);
    myFile.print(",");
    myFile.print(gy);
    myFile.print(",");
    myFile.print(gz);
    myFile.print(launchTime);
    myFile.print(",");
    myFile.print(ApoTime);
    myFile.print(",");
    myFile.println(landTime);

  } else {
    #ifdef SERIAL_DEBUG
    Serial.println(F("F-0"));
    #endif
  }
}


// ================================================================
// ===                  MISC FUNCTIONS                          ===
// ================================================================


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

void GREEN() {
  //Everything is fine.. signal.
  unsigned long interval = 1000;
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis > interval) {
    previousMillis = currentMillis;
    
    digitalWrite(GLED, HIGH);
    tone(buzzer, 2500, 100);
  }
  else {
    digitalWrite(GLED, LOW);
  }
}

void get_alt() {
  bmp280.getAltitude(alt);
  bmp280.getPressure(pascal);
  est_alt = pressureKalmanFilter.updateEstimate(alt);

  return est_alt;
}