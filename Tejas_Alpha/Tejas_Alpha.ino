// =============================
// == PHASE ZERO & ONE Devs ====
//==============================

/*Tejas Alpha - @Nikhil Mishra
 * https://curious-nikhil.github.io/
 * Tejas is a rocket computer  
 * Tejas controls the pitch and the roll of the rocket
 */  
// =============================
// ==     CONFIGURATION       ==
//==============================
//DATE
#define DAY 01
#define MONTH 07
#define YEAR 2019
#define VERSION 0.1

#define PID_MOD
//#define SERVO_MOD
#define SERIAL_DEBUG


// =============================
// == Include and Define Vars ==
//==============================

//Header Files
#include <PID_v1.h>
#include <Servo.h> //servo library
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h> // Gyroscope and axcelerometer libraries
#include <Wire.h>
#include "i2c.h"
#include "i2c_BMP280.h"
#include "SimpleKalmanFilter.h"

//SD Card Libraries
#include <SPI.h>
#include <SD.h>


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define RLED 6// Green LED
#define GLED 7// Green LED


bool blinkState = true;
float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;

#ifdef SERVO_MOD
  Servo ServoX;   // X axis Servo
  Servo ServoY;   // Y axis Servo
#endif

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer


Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


#define PITCH   1     // defines the position within ypr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define ROLL  2     // defines the position within ypr[x] variable for ROLL; may vary due to sensor orientation when mounted
#define YAW   0     // defines the position within ypr[x] variable for YAW; may vary due to sensor orientation when mounted

//PID vars
double SetpointX, InputX, OutputX;
double SetpointY, InputY, OutputY;

//Gain Vals for X and Y
double KpX = 3.55, KiX = 0.005, KdX = 2.05;
double KpY = 3.55, KiY = 0.005, KdY = 2.05;


//Parsing PID Input Vals
#ifdef PID_MOD
  PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);
  PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);
#endif

MPU6050 mpu;

//Calibration 

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;


// ================================================================
// ===              BAROMETER                                   ===
// ================================================================
#define GLED 9
#define RLED 6
static float meters;
float temperature;
float pascal;
float est_alt;
bool err = false;
String filename; 
File dataFile;

BMP280 bmp280;
SimpleKalmanFilter pressureKalmanFilter(1, 1, 0.01);


//INTERRUPT DETECTION ROUTINE            
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
  void dmpDataReady() {
      mpuInterrupt = true;
  }

// ================================================================
// ===               THE SETUP FUNCTION                       ===
void setup() {

  Serial.begin(38400);
  Serial.println("I got to Setup");
  
  #ifdef SERVO_MOD
    //Attach the servos
    ServoX.attach(8);
    ServoY.attach(10);
  
    //Set the Servos to 90 Degrees
    ServoX.write(45);
    delay(1000);
    ServoY.write(45);
  #endif
  
    //Offset Vals
    ax_offset = 1118;
    ay_offset = 513;
    az_offset = 1289;
    gx_offset = 64;
    gy_offset = -1;
    gz_offset = -33;
  
    // configure LED for output
    //pinMode(LED_PIN, OUTPUT);
    pinMode(GLED, OUTPUT);
    pinMode(RLED, OUTPUT);

    //Initialize SD Module
    //initializeSD();
  
    //Initialize Gyroscope and Servo
    //initializeDMP();
  
    //Initialize Baromter
    //initializeBMP();

//------------------------------------------------------------------------------//
     Serial.print("Initializing SD card...");

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  if (SD.exists("example.txt")) {
    Serial.println("example.txt exists.");
  } else {
    Serial.println("example.txt doesn't exist.");
  }

  // open a new file and immediately close it:
  Serial.println("Creating example.txt...");
  dataFile = SD.open("dlog.txt", FILE_WRITE);
  if (dataFile) {
      //Print Date in File
      dataFile.print("V ");
      dataFile.println(VERSION);
      
      dataFile.print("Date: ");
      dataFile.print(DAY);
      dataFile.print(",");      
      dataFile.print(MONTH);
      dataFile.print(",");
      dataFile.println(YEAR);
  
      //Print Header Files  - - meters, pascal, est_alt, mpuPitch, mpuRoll, mpuYaw, OutputX, OutputY

      dataFile.print("Height");
      dataFile.print(",");
      dataFile.print("Pascal");
      dataFile.print(",");
      dataFile.print("Kalman_Height");
      dataFile.print(",");
      dataFile.print("mpuPitch");
      dataFile.print(",");
      dataFile.print("mpuRoll");
      dataFile.print(",");
      dataFile.print("mpuYaw");
      dataFile.print(",");
      dataFile.print("OutputX");
      dataFile.print(",");
      dataFile.println("OutputY");
 
      dataFile.close();
      Serial.println("File Created and File Closed");

  } else {
    dataFile.print("Why is luck so bad?");
    while(1);
  }

    
}

// ================================================================
// ===                INITIALIZE FUNCTION                       ===
// ================================================================

// ================================================================
// ===               SD CARD Begin                       ===
// ================================================================

void initializeSD(){
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
      while(1);
    }
    else {
      Serial.println("File name created!");
    }

    dataFile = SD.open("test.txt", FILE_WRITE);
    
    if (dataFile) {      
      //Print Date in File
      dataFile.print("V ");
      dataFile.println(VERSION);
      
      dataFile.println("Date: ");
      dataFile.print(DAY);
      dataFile.print(MONTH);
      dataFile.println(YEAR);
  
      //Print Header Files  - - meters, pascal, est_alt, mpuPitch, mpuRoll, mpuYaw, OutputX, OutputY

      dataFile.print("Height");
      dataFile.print(",");
      dataFile.print("Pascal");
      dataFile.print(",");
      dataFile.print("Kalman_Height");
      dataFile.print(",");
      dataFile.print("mpuPitch");
      dataFile.print(",");
      dataFile.print("mpuRoll");
      dataFile.print(",");
      dataFile.print("mpuYaw");
      dataFile.print(",");
      dataFile.print("OutputX");
      dataFile.print(",");
      dataFile.println("OutputY");
 
      dataFile.close();
      Serial.println("File Created and File Closed");

    } else {
      #ifdef SERIAL_DEBUG
        Serial.println("Error opening file");
      #endif
      digitalWrite(RLED, HIGH);
      while(1);
    }
    
}

// ================================================================
// ===                         BAROMETER                       ===
// ================================================================

void initializeBMP() {
        
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

}


// =========================================================
// ===                          DMP                      ===
// ==========================================================

void initializeDMP() {

  Wire.begin();
  
  Serial.begin(38400);
  while (!Serial);

  pinMode(INTERRUPT_PIN, INPUT);

   // initialize device
  Serial.println(F("Tejas..... Alpha!"));
  Serial.println(F("Initializing MPU 6050 device..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
      
    // wait for ready
    Serial.println(F("\nSend any character to Begin Tejas Demo! :D "));
    while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();

  
  //Supply Gyro Calibration Values
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);

  
    //PID
    myPIDX.SetOutputLimits(-30, 30);
    myPIDY.SetOutputLimits(-30, 30);
    myPIDX.SetMode(AUTOMATIC);
    myPIDY.SetMode(AUTOMATIC);
    SetpointX = 0;
    SetpointY = 0;

  if (devStatus == 0) {
    
    //Turn on the DMP
    Serial.println(F("Enabling DMP....."));
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();

  }
  else
  {
    // ERROR!   
    // 1 = initial memory load failed, 2 = DMP configuration updates failed (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed code = "));
    Serial.println(devStatus);
  }

 //Initialization Done Signal!

}

// ================================================================
// ===                    MAIN LOOP                            ===
// ================================================================
void loop() {
  //altimeter();
  //tejas_move();

  //Data logging
  //writeSD(meters, pascal, est_alt, mpuPitch, mpuRoll, mpuYaw, OutputX, OutputY);

}

// ================================================================
// ===                   TEJAS FUNCTION                         === 
// ===               DMP + PID + SERVO WRITE                    ===
// ================================================================

void tejas_move(void) {
  //DMP Program
    // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
      if (mpuInterrupt && fifoCount < packetSize) {
        // try to get out of the infinite loop 
        fifoCount = mpu.getFIFOCount();
      }  
  }

  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & _BV(MPU6050_INTERRUPT_FIFO_OFLOW_BIT)) || fifoCount >= 1024) {
    
    // reset so we can continue cleanly
    mpu.resetFIFO();
    fifoCount = mpu.getFIFOCount();
    Serial.println(F("FIFO overflow!"));
    
  } else if (mpuIntStatus & _BV(MPU6050_INTERRUPT_DMP_INT_BIT)){
            // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        //MPU Computation
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        mpuPitch = ypr[PITCH] * 180 / M_PI;
        mpuRoll = ypr[ROLL] * 180 / M_PI;
        mpuYaw  = ypr[YAW] * 180 / M_PI;

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(GLED, blinkState);
        mpu.resetFIFO();
        InputX = mpuPitch;
        myPIDX.Compute();
        InputY = mpuRoll;
        myPIDY.Compute();

        #ifdef SERVO_MOD 
            ServoX.write(-mpuPitch + 90);
            ServoY.write(mpuRoll + 90);
        #endif

        #ifdef SERIAL_DEBUG
          Serial.print(mpuPitch);
          Serial.print("    ");
          Serial.print(mpuRoll);
          Serial.print("    ");
          Serial.print(OutputX);
          Serial.print("    ");
          Serial.println(OutputY);
        #endif
          
  }
}


// ================================================================
// ===           ALTIMETER - BAROMETER & KALMAN FILTER          ===
// ================================================================

void altimeter() {

    bmp280.setEnabled(0);
    bmp280.triggerMeasurement();

    bmp280.getTemperature(temperature);  // throw away - needed for alt.
    bmp280.getPressure(pascal);     // throw away - needed for alt.
    bmp280.getAltitude(meters);

    float est_alt = pressureKalmanFilter.updateEstimate(meters);

    Serial.print(meters);
    Serial.print(",");
    Serial.print(pascal);
    Serial.print(",");
    Serial.println(est_alt);
}
// ================================================================
// ===           SD CARD WRITE AND STUFF                        ===
// ================================================================
//Create a new filename everytime.
boolean loadSDFile() {
  int i = 0;
  boolean file = false;

  while(!file && i < 1024) {
    filename = (String)i + "dlog.txt";

    if (!SD.exists(filename)) {
      dataFile = SD.open(filename, FILE_WRITE);
      delay(10);
      dataFile.close();
      file = true;
    }
    i++;
  }

  return file;
}
// meters, pascal, est_alt, mpuPitch, mpuRoll, mpuYaw, OutputX, OutputY
void writeSD(float meters, float pascal, float est_alt, float mpuPitch, float mpuRoll, float mpuYaw, float OutputX, float OutputY) {

  dataFile = SD.open(filename, FILE_WRITE);

  if (dataFile) {
    //Serial Prints

    #ifdef SERIAL_DEBUG
      Serial.print(meters);
      Serial.print(",");
      Serial.print(pascal);
      Serial.print(",");
      Serial.println(est_alt);
      Serial.print(",");
      Serial.print(mpuPitch);
      Serial.print(",");
      Serial.print(mpuRoll);
      Serial.print(",");
      Serial.print(mpuYaw);
      Serial.print(",");  
      Serial.println(OutputX);
      Serial.print(",");
      Serial.println(OutputY);  
    #endif

    //Writing in SD Card!
    dataFile.print(meters);   
    dataFile.print(",");
    dataFile.print(pascal);
    dataFile.print(",");
    dataFile.print(est_alt);
    dataFile.print(mpuPitch);   
    dataFile.print(",");
    dataFile.print(mpuRoll);
    dataFile.print(",");
    dataFile.print(mpuYaw);
    dataFile.print(",");
    dataFile.print(OutputX);
    dataFile.print(",");
    dataFile.println(OutputY);
    
    dataFile.close();
  
    delay(100);

  } else {
    #ifdef SERIAL_DEBUG
      Serial.println("error Opening the txt file");
    #endif
    err = true;
    while(1);
  }
}


// ================================================================
// ===                  MISC FUNCTIONS                          ===
// ================================================================
