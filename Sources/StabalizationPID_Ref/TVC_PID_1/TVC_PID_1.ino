//https://github.com/bdureau/RocketMotorPIDGimbal

/*
   Description: Model Rocket motor gimbal using 2 servo. This is yet again another attempt to fly a model rocket without fins using long burn motors.
                It is using the Arduino PID controler to move a rocket motor.
                Angle changes can be monitored using a USB cable or a bluetooth interface
                Inspired by various camera gimbal projects
   Author: Boris du Reau
   Date: June 2018
   Sensor used is an MPU6050 board
   You can use an Arduino Uno/Nano or stm32F103C board
   Servo Connection
   BROWN - gnd
   red - 5v
   yellow - d10 (pwm for Sero X) or PA1 for stm32
          - d11 (servo Y) or PA2 for stm32
   MPU board Connection
   VCC - 5v
   GND - GND
   SCL - A5  or pin SCL on the stm32
   SDA - A4  or pin SDA on the stm32
   INT - D2 (not used)
  TODO:
  Build an Android interface to monitor the telemetry and configure it
*/


#include <Servo.h> //servo library
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h> // Gyroscope and axcelerometer libraries
#include <PID_v1.h> // Arduino PID library
#include <Wire.h>

#define LED_PIN PC13 //pin 13 for the arduino Uno and PC13 for the stm32 
bool blinkState = true;

Servo ServoX;   // X axis Servo
Servo ServoY;   // Y axis Servo

float mpuPitch = 0;
float mpuRoll = 0;
float mpuYaw = 0;


// Create  MPU object
// class default I2C address is 0x68; specific I2C addresses may be passed as a parameter here
// for exemple MPU6050 mpu(0x69);
MPU6050 mpu;


// MPU control/status vars
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// relative ypr[x] usage based on sensor orientation when mounted, e.g. ypr[PITCH]
#define PITCH   1     // defines the position within ypr[x] variable for PITCH; may vary due to sensor orientation when mounted
#define ROLL  2     // defines the position within ypr[x] variable for ROLL; may vary due to sensor orientation when mounted
#define YAW   0     // defines the position within ypr[x] variable for YAW; may vary due to sensor orientation when mounted

// PID stuff
//Define Variables we'll be connecting to
double SetpointX, InputX, OutputX;
double SetpointY, InputY, OutputY;

// Those initial tuning parameters need to be tuned
// this a bit like when you tune your copter appart from the fact that the rocket motor last only few seconds
// please help !!!!!!!
//double KpX = 2, KiX = 5, KdX = 1;
double KpX = 3.55, KiX = 0.005, KdX = 2.05;
//Specify the links and initial tuning parameters
//double KpY = 2, KiY = 5, KdY = 1;
double KpY = 3.55, KiY = 0.005, KdY = 2.05;
PID myPIDX(&InputX, &OutputX, &SetpointX, KpX, KiX, KdX, DIRECT);
PID myPIDY(&InputY, &OutputY, &SetpointY, KpY, KiY, KdY, DIRECT);


//calibration stuff
//Change those 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 200;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)

int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)

int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int16_t ax, ay, az, gx, gy, gz;

int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

/*
   Initial setup
   do the board calibration
   if you use the calibration function do not move the board until the calibration is complete
*/

void setup()
{

  ServoX.attach(PA1);  // attaches the X servo on PA1 for stm32 or D10 for the Arduino Uno
  ServoY.attach(PA2);  // attaches the Y servo on PA2 for stm32 or D11 for the Arduino Uno

  // set both servo's to 90 degree
  ServoX.write(90);
  ServoY.write(90);
  delay(500);

  Wire.begin();


  Serial1.begin(38400);
  while (!Serial1);      // wait for Leonardo enumeration, others continue immediately

  // initialize device
  Serial1.println(F("Initializing MPU 6050 device..."));
  mpu.initialize();

  // verify connection
  Serial1.println(F("Testing device connections..."));
  Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial1.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();


  // INPUT CALIBRATED OFFSETS HERE; SPECIFIC FOR EACH UNIT AND EACH MOUNTING CONFIGURATION!!!!
  // use the calibrate function for yours
  // you can also write down your offset ans use them so that you do not have to re-run the calibration

  ax_offset = 1118;
  ay_offset = 513;
  az_offset = 1289;
  gx_offset = 64;
  gy_offset = -1;
  gz_offset = -33;
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  calibrate();
  initialize();
}

void initialize() {
  // initialize device
  Serial1.println(F("Initializing MPU 6050 device..."));
  mpu.initialize();

  // verify connection
  Serial1.println(F("Testing device connections..."));
  Serial1.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // load and configure the DMP
  Serial1.println(F("Initializing DMP"));
  devStatus = mpu.dmpInitialize();
  mpu.setXAccelOffset(ax_offset);
  mpu.setYAccelOffset(ay_offset);
  mpu.setZAccelOffset(az_offset);
  mpu.setXGyroOffset(gx_offset);
  mpu.setYGyroOffset(gy_offset);
  mpu.setZGyroOffset(gz_offset);
  
  //turn the PID on
  //servo's can go from 60 degree's to 120 degree so set the angle correction to + - 30 degrees max
  myPIDX.SetOutputLimits(-30, 30);
  myPIDY.SetOutputLimits(-30, 30);
  myPIDX.SetMode(AUTOMATIC);
  myPIDY.SetMode(AUTOMATIC);
  SetpointX = 0;
  SetpointY = 0;
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    Serial1.println(F("Enabling DMP"));
    mpu.setDMPEnabled(true);

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed, 2 = DMP configuration updates failed (if it's going to break, usually the code will be 1)
    Serial1.print(F("DMP Initialization failed code = "));
    Serial1.println(devStatus);
  }
}


/*
   MAIN PROGRAM LOOP
*/
void loop(void)
{
  MainMenu();
  //myloop();
}

void myloop(void)
{
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ( fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial1.println(F("FIFO overflow!"));
    return;
  }

  // check for correct available data length
  if (fifoCount < packetSize)
    return;

  // read a packet from FIFO
  mpu.getFIFOBytes(fifoBuffer, packetSize);

  // track FIFO count here in case there is > 1 packet available
  fifoCount -= packetSize;

  // flush buffer to prevent overflow
  mpu.resetFIFO();

  // display Euler angles in degrees
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  mpuPitch = ypr[PITCH] * 180 / M_PI;
  mpuRoll = ypr[ROLL] * 180 / M_PI;
  mpuYaw  = ypr[YAW] * 180 / M_PI;

  // flush buffer to prevent overflow
  mpu.resetFIFO();

  // blink LED to indicate activity
  blinkState = !blinkState;
  digitalWrite(LED_PIN, blinkState);

  // flush buffer to prevent overflow
  mpu.resetFIFO();
  InputX = mpuPitch;
  myPIDX.Compute();
  InputY = mpuRoll;
  myPIDY.Compute();

  //if using PID do those
  ServoX.write(-OutputX + 90);
  ServoY.write(OutputY + 90);

  // if you do not want to use the PID
  // ServoX.write(-mpuPitch + 90);
  // ServoY.write(mpuRoll + 90);

  float q1[4];
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  q1[0] = q.w;
  q1[1] = q.x;
  q1[2] = q.y;
  q1[3] = q.z;
  serialPrintFloatArr(q1, 4);
  Serial1.print(mpuPitch);
  Serial1.print("    ");
  Serial1.print(mpuRoll);
  Serial1.print("    ");
  Serial1.print(OutputX);
  Serial1.print("    ");
  Serial1.println(OutputY);
  delay(10);

  // flush buffer to prevent overflow
  mpu.resetFIFO();

}

// ================================================================
// === Those 2 functions will format the data                   ===
// ================================================================
void serialPrintFloatArr(float * arr, int length) {
  for (int i = 0; i < length; i++) {
    serialFloatPrint(arr[i]);
    Serial1.print(",");
  }
}


void serialFloatPrint(float f) {
  byte * b = (byte *) &f;
  for (int i = 0; i < 4; i++) {

    byte b1 = (b[i] >> 4) & 0x0f;
    byte b2 = (b[i] & 0x0f);

    char c1 = (b1 < 10) ? ('0' + b1) : 'A' + b1 - 10;
    char c2 = (b2 < 10) ? ('0' + b2) : 'A' + b2 - 10;

    Serial1.print(c1);
    Serial1.print(c2);
  }
}


/*
   calibration routines those will be executed each time the board is powered up.
   we might want to calibrate it for good on a flat table and save it to the microcontroler eeprom
*/
void calibrate() {
  // start message
  Serial1.println("\nMPU6050 Calibration Sketch");
  //delay(1000);
  Serial1.println("\nYour MPU6050 should be placed in horizontal position, with package letters facing up. \nDon't touch it until you see a finish message.\n");
  //delay(1000);
  // verify connection
  Serial1.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  //delay(1000);
  // reset offsets
  mpu.setXAccelOffset(0);
  mpu.setYAccelOffset(0);
  mpu.setZAccelOffset(0);
  mpu.setXGyroOffset(0);
  mpu.setYGyroOffset(0);
  mpu.setZGyroOffset(0);

  while (1) {
    if (state == 0) {
      Serial1.println("\nReading sensors for first time...");
      meansensors();
      state++;
      delay(100);
    }

    if (state == 1) {
      Serial1.println("\nCalculating offsets...");
      calibration();
      state++;
      delay(100);
    }

    if (state == 2) {
      meansensors();
      Serial1.println("\nFINISHED!");
      Serial1.print("\nSensor readings with offsets:\t");
      Serial1.print(mean_ax);
      Serial1.print("\t");
      Serial1.print(mean_ay);
      Serial1.print("\t");
      Serial1.print(mean_az);
      Serial1.print("\t");
      Serial1.print(mean_gx);
      Serial1.print("\t");
      Serial1.print(mean_gy);
      Serial1.print("\t");
      Serial1.println(mean_gz);
      Serial1.print("Your offsets:\t");
      Serial1.print(ax_offset);
      Serial1.print("\t");
      Serial1.print(ay_offset);
      Serial1.print("\t");
      Serial1.print(az_offset);
      Serial1.print("\t");
      Serial1.print(gx_offset);
      Serial1.print("\t");
      Serial1.print(gy_offset);
      Serial1.print("\t");
      Serial1.println(gz_offset);
      Serial1.println("\nData is printed as: acelX acelY acelZ giroX giroY giroZ");
      Serial1.println("Check that your sensor readings are close to 0 0 16384 0 0 0");
      Serial1.println("If calibration was succesful write down your offsets so you can set them in your projects using something similar to mpu.setXAccelOffset(youroffset)");
      //while (1);
      break;
    }
  }
}


void meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    Serial1.println("...");
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
}

//================================================================
// Main menu to interpret all the commands sent by the altimeter console
//================================================================
void MainMenu()
{
  char readVal = ' ';
  int i = 0;

  char commandbuffer[200];



  while (Serial1.available())
  {
    readVal = Serial1.read();
    if (readVal != ';' )
    {
      if (readVal != '\n')
        commandbuffer[i++] = readVal;
    }
    else
    {
      commandbuffer[i++] = '\0';
      break;
    }
  }

  if (commandbuffer[0] != '\0') {
    interpretCommandBuffer(commandbuffer);
    commandbuffer[0] = '\0';
  }
  myloop();
}

void interpretCommandBuffer(char *commandbuffer) {
  //Serial1.println((char*)commandbuffer);
  //this will erase all flight
  if (commandbuffer[0] == 'c')
  {
    Serial1.println(F("calibration\n"));
    // Do calibration suff
    state = 0;

    calibrate();
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);
    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

  }
  else
  {
    Serial1.println(F("Unknown command" ));
    Serial1.println(commandbuffer[0]);
  }
}
