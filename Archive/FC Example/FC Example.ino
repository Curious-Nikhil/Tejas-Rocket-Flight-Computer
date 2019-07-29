/*  Altwizz 
 *  Ver 0.4
 *  
 *  Arduino flight logger/computer 
 *  
 *  by Justin Bray 
*/

//Included librarys
#include <MPU6050.h>
#include <Wire.h>
#include <MS5611.h>
#include <SD.h>

//***Global variables***

//Define barometer sensor
MS5611 ms5611;
int stoc = 0; //Sensor try counter
bool sensorTimeout = false;

//Define accelerameter
MPU6050 accelgyro;
int16_t ax, ay, az;


double referencePressure; //Starting pressure
unsigned long time; //System uptime
float alt;   //Storing current altitude
float lastAlt; //Storing previous altitude
int buzzer = 6; //Pin for buzzer

//SD card variables
File dataFile; 
String xmlFile; 
const int chipSelect = 4; //Chip slect for SD card




bool err = false; //Error check

//Variables for charges and safety checks
bool armCharges = false;
bool apogeeFire = false;
bool mainLock = false;
int mainCount = 0;
bool mainFire = false;
int apogeeFireTime;
int mainFireTime;
int apogeeCount = 0;
bool apogeeLock = false;

//Output pins for pyro charges
int apogeePin = 9;
int mainPin = 3;
int stage2Pin = 8;

//Altitude to fire main chute (Multiplied by 0.3048 to convert feet to meters.)
int mainAlt = 250 * 0.3048;

//Setup code
void setup() 
{
  //Configure pyro pins for output
  pinMode(mainPin, OUTPUT);
  pinMode(apogeePin, OUTPUT);
  pinMode(stage2Pin, OUTPUT);

  //Start searial connection (For debugging)
  Serial.begin(9600); // start serial for output
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  //SD card initialization
  Serial.println("Setup");
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    err = true;
  }
  //Call function to detect next file name and create blank file
  if(!loadSDFile()){
    Serial.println("File Failure.");
    err = true;
  }else{
    Serial.println("Done.");
  }

  // Initialize MS5611 sensor
  Serial.println("Initialize MS5611 Sensor");

  while(!ms5611.begin() && !sensorTimeout)
  {
    Serial.println("Could not find a valid MS5611 sensor, check wiring!");
    delay(500);
    stoc++;
    if(stoc = 5){
      sensorTimeout = true;
    }
    
  }

  // Get reference pressure for relative altitude
  referencePressure = ms5611.readPressure();

  // Check settings
  Serial.print("Oversampling: ");
  Serial.println(ms5611.getOversampling());

  // Initialize MPU6050 sensor
  accelgyro.initialize();
  if(!accelgyro.testConnection()){
    Serial.println("Gyro Failed!");
    err = true;
  }

  //Give 3 beeps if setup was successful or 1 long beep if there was an error
  if(err){
    analogWrite(buzzer, 97);
    delay(3000);
    analogWrite(buzzer, 0);
    Serial.println("Err!");
  }else{
    int j = 0;
    while(j<3){
      analogWrite(buzzer, 97);
      delay(500);
      analogWrite(buzzer, 0);
      delay(500);
      j++;
    }
    
  }
  
}

void loop()
{
  //Get current alt
  uint32_t rawPressure = ms5611.readRawPressure();
  long realPressure = ms5611.readPressure();
  alt = ms5611.getAltitude(realPressure, referencePressure);
  Serial.print(alt);    
  Serial.println(" m");

  //Get current acceleration
  accelgyro.getAcceleration(&ax, &ay, &az);

  //Get current system up time
  time = millis();

  //Write data to SD card file
  writeSD(alt,time,ay,mainFire,apogeeLock); 


  //Launch safety check (Rocket must be above 200ft.{67m} to arm any pyro charges)
  if(armCharges){
    apogeeCheck();
  }else{
    if(alt > 67){
      armCharges = true;
    }
  }
  lastAlt = alt; //Set current alt to last alt
  
}

//Must get 3 reading lower than the last reading in a row to be considered apogee
void apogeeCheck(){
  if(alt < lastAlt){
    apogeeCount = apogeeCount + 1;
  } else {
    apogeeCount = 0;
  }
  if(apogeeCount > 2 || apogeeLock){
    apogeeLock = true;
    charges(); 
  }
  
}

void charges(){
  //If apogee hasn't fired fire now
  if(!apogeeFire){
    apogeeFire = true;
    apogeeFireTime = time; //Set time apogee charge was fired
    digitalWrite(apogeePin, HIGH); //Fire apogee charge
  }else{
    //After 3 seconds stop apogee charge
    if(apogeeFireTime + 3000 < time){
        digitalWrite(apogeePin, LOW); 
    }
  }

  //Check if current alt is less than set main deploy alt for 3 reading
  //***IMPORTANT*** When apogee fires there is a spike in the reading that fires main imidiatley after
  if(mainAlt > alt){
    mainCount++;
    if(mainCount == 3){
      mainLock = true;
    }
  }else{
    mainCount = 0;
  }
  if(mainLock){
    //If main hasn't fired fire now
    if(!mainFire){
      mainFire = true;
      mainFireTime = time; //Set time main charge was fired
      digitalWrite(mainPin, HIGH); //Fire main charge
    }else{
      //After 3 seconds stop main charge
      if(mainFireTime + 3000 < time){
        digitalWrite(mainPin, LOW);
      }
    }
  }
}

//Function to calculate file name by next number and create a blank file with .AWD extension
boolean loadSDFile(){
    int i = 0;
    boolean file = false;
    //Search through files in until the file with highest number for it's name is found then create new file with next number
    //Files will only go up to 1024
    while(!file && i < 1024){
       xmlFile = (String)i + ".awd";
       if (!SD.exists(xmlFile)){
          dataFile = SD.open(xmlFile, FILE_WRITE);
          delay(1000);          
          dataFile.close();
          file = true;
       }
       i++;
    }
    return file;
}

//Function to write data to the SD Card in a CSV format
void writeSD(float a, long t, int16_t x, bool mf, bool al){
  int m = 0;
  if(mf){
    m = 1;
  }
  int alck = 0;
  if(al){
    alck =  1;
  }
    dataFile = SD.open(xmlFile, FILE_WRITE);
     dataFile.println((String)a + "," + (String)t + "," + (String)x + "," + (String)m + "," + (String)alck);
     dataFile.close(); //Must be closed after write or the file will not save
    
}

void soundBuzzer(long t){
  int seconds = t/1000;
  Serial.println(seconds);
  if((seconds % 2) == 0){
    analogWrite(buzzer, 97);
    Serial.println("buzz on");
  }else{
    analogWrite(buzzer, 0);
    Serial.println("buzz off");
  }
}
