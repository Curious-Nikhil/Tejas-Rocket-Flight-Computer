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


//#define SERVO_MOD
//#define DMP_MOD
#define SERIAL_DEBUG
//#define MPU_CALBIRATION
// =============================
// == Include and Define Vars ==
//==============================

//Header Files
#include "i2c.h"
#include "i2c_BMP280.h"
#include "MPU6050.h"
#include <Wire.h>
#include "SimpleKalmanFilter.h"
#include <SD.h>


#define INTERRUPT_PIN 2  // 
#define RLED 6// Green LED
#define GLED 7// Green LED
#define buzzer 8
#define pyroPin 9
// =============================================
// ===          MISC Global Vars             ===
// =============================================

//Timers Vars
unsigned long previousMillis = 0;
unsigned long currentMillis;
bool launch = 0;
bool pyro = false;
bool pyroFired = false;
bool landed = false;
bool ABORT = false;

int lastAlt = 0;

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
  tone(buzzer, 2500, 300);
  delay(1000);
  tone(buzzer, 2000, 300);
  delay(500);
  tone(buzzer, 3000, 300);
  tone(buzzer, 3500, 300);
  delay(500);
  tone(buzzer, 2500, 300);
  delay(1000);
  delay(2000);

  // configure LED for output

  pinMode(GLED, OUTPUT);
  pinMode(RLED, OUTPUT);
  pinMode(pyroPin, OUTPUT);

  Serial.println(FreeRam()); //set a threshold for that.

  //PASS 1 -- confirm the size.
  if (FreeRam() < 275) {
    Serial.println(F("RAM-0"));
    RED();
    while (1);
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
  float sum = 0;
  digitalWrite(GLED, HIGH);
  delay(5000);
  digitalWrite(GLED, LOW);

  for (int i = 0; i < 30; i++) {
    digitalWrite(GLED, LOW);
    digitalWrite(RLED, HIGH);

    delay(100);

    digitalWrite(GLED, HIGH);
    digitalWrite(RLED, LOW);

    bmp280.getAltitude(base_alt);
    sum += base_alt;
    Serial.println(base_alt);
  }

  base_alt = sum / 30.0;
  Serial.print(F("BASEH: "));
  Serial.println(base_alt);

  delay(1000);
  digitalWrite(GLED, LOW);
  digitalWrite(RLED, LOW);
}

// ================================================================
// ===                    MAIN LOOP                            ===
// ================================================================
void loop() {
  //Disable Pyros
  digitalWrite(pyroPin, LOW);

  if (launch == false && pyro == false && landed == false) {
    GREEN();
  }
  else if (landed == true) {
    LAND_SIG();
  }
  

  /*************
     Detect launch
     Enable ABORT - for extreame tilt.
     Detect Apogee
   * * Fire Pyros
     Flight Log
  */


  get_alt();
  motion();

  if (launch == 0) {
    //Serial.println(ay);
  }

  if (ay > 30000 || launch == true && landed == false) {

    if (launch == 0) {
      Serial.println(F("LAUNCH! ! !"));
      //launchTime = millis();
      delay(1000);
    }
    Serial.println("LAUNCH");

    launch = 1;

    //APOGEE DETECTION PROGRAM
    get_alt();

    Serial.println(est_alt - lastAlt);

    if (est_alt - lastAlt <= -0.5 && pyro == false && launch == true && pyroFired == false) {
      //check for drop
      //Store time of Apogee Trigger 1
      delay(20);
      get_alt();
      Serial.println(F("P1"));
      tone(buzzer, 2500, 200);

      if (est_alt - lastAlt <= -1 && pyro == false && launch == true && pyroFired == false) {
        //check for  drop
        //Store time of Apogee Trigger 1

        delay(20);
        get_alt();
        Serial.println(F("P2"));
        tone(buzzer, 2500, 200);

        if (est_alt - lastAlt <= -2 && pyro == false && launch == true && pyroFired == false) {
          //PASS 3
          //Store time of Apogee Pyro Fire
          //Fire Pyros!
          pyro = true;
          Serial.println(F("P3"));
          tone(buzzer, 2500, 200);

        } else {
          lastAlt = est_alt;
        }
      } else {
        lastAlt = est_alt;
      }
    }
    else {
      lastAlt = est_alt;
    }
  }

  if (pyro == true && launch == true && pyroFired == false) {

    Serial.println(F("pyro"));
    digitalWrite(RLED, HIGH);
    tone(buzzer, 2500, 1000);
    currentMillis = millis();

    if (currentMillis - previousMillis >= 2000) {
      pyroFired = true;

      previousMillis = currentMillis;
    }

  }

  
  //&& pyro == true && pyroFired == true
  //( 0 < est_alt - base_alt < 3 )

  // if (( 0 < est_alt - base_alt < 2 ) && launch == true) {
  //   //Land PROGRAM
  //   landed = true;

  //   Serial.println("LANDED!");
  // }

  //Flight Logs

  if (launch == true && landed == false) {
    //Flight Logs

    Write();
    sd_count++;

    if (sd_count > 100) {
      myFile.flush();
      sd_count = 0;
      Serial.println("FLUSH");

    }

    if (landed == true) {
      myFile.close();
    }

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
    myFile.print("alt");
    myFile.print(",");
    myFile.print("KMF");
    myFile.print(",");
    myFile.print("ax");
    myFile.print(",");
    myFile.print("ay");
    myFile.print(",");
    myFile.print("az");
    myFile.print(",");
    myFile.print("LaunchTime");
    myFile.print(",");
    myFile.print("ApoTime");
    myFile.print(",");
    myFile.println("LandTime");

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
  accelgyro.getAcceleration(&ax, &ay, &az);
}

// ================================================================
// ===           SD CARD WRITE AND STUFF                        ===
// ================================================================
//Create a new filename everytime.
boolean loadSDFile() {
  int i = 0;
  boolean file = false;

  while (!file && i < 1024) {
    filename = (String)i + "FL.csv";

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

void Write() {
  myFile = SD.open(filename, FILE_WRITE);

  Serial.println(FreeRam());

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
    myFile.print(launch);
    myFile.print(",");
    myFile.print(pyro);
    myFile.print(",");
    myFile.println(landed);


    myFile.close();

    //delay(10);

  } else {
#ifdef SERIAL_DEBUG
    Serial.println(F("FW-0"));
#endif

    while (1);
  }
}

// ================================================================
// ===                  MISC FUNCTIONS                          ===
// ================================================================


void RED()
{
  digitalWrite(RLED, HIGH);
  tone(buzzer, 2500, 100);
  delay(200);
  digitalWrite(RLED, LOW);

  tone(buzzer, 2500, 100);
  delay(200);
  digitalWrite(RLED, HIGH);

  tone(buzzer, 2000, 100);
  delay(500);
  tone(buzzer, 2000, 100);
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

void LAND_SIG() {
  //Everything is fine.. signal.
  unsigned long interval = 100;
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
}
