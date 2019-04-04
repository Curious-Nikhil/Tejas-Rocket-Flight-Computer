
#include <PID_v1.h>
#include <Servo.h> //servo library
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h> // Gyroscope and axcelerometer libraries
#include <Wire.h>

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
#define green 12// Green LED

bool blinkState = true;
float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;


Servo ServoX;   // X axis Servo
Servo ServoY;   // Y axis Servo

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
PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);


MPU6050 mpu;

//Calibration 

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

void setup() {

  Serial.println(F("Tejas..... Alpha!"));

  //Attach the servos
  ServoX.attach(8);
  ServoY.attach(10);

  //Set the Servos to 90 Degrees
  ServoX.write(45);
  delay(1000);
  ServoY.write(45);


  //Offset Vals
  ax_offset = 1118;
  ay_offset = 513;
  az_offset = 1289;
  gx_offset = 64;
  gy_offset = -1;
  gz_offset = -33;

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(green, OUTPUT);

  initialize();
  
}

void initialize() {

  Wire.begin();
  
  Serial.begin(38400);
  while (!Serial);

  pinMode(INTERRUPT_PIN, INPUT);


   // initialize device
  Serial.println(F("Initializing MPU 6050 device..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

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
 green_fn();
}
void loop() {

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
            digitalWrite(green, blinkState);
            mpu.resetFIFO();
            InputX = mpuPitch;
            myPIDX.Compute();
            InputY = mpuRoll;
            myPIDY.Compute();

             ServoX.write(-mpuPitch + 90);
             ServoY.write(mpuRoll + 90);

              Serial.print(mpuPitch);
              Serial.print("    ");
              Serial.print(mpuRoll);
              Serial.print("    ");
              Serial.print(OutputX);
              Serial.print("    ");
              Serial.println(OutputY);

             
  }

}

void green_fn() {

  digitalWrite(green, HIGH);   
  delay(100);                       
  digitalWrite(green, LOW);    
  delay(100);
   digitalWrite(green, HIGH);   
  delay(100);                       
  digitalWrite(green, LOW);    
  delay(100);                        
}
