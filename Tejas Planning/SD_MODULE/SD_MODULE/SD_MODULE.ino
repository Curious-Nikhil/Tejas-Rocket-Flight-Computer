/*
  SD card basic file example

 This example shows how to create and destroy an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4 (for MKRZero SD: SDCARD_SS_PIN)

 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */
//#include <SPI.h>
#include <SD.h>
String filename; 
File myFile;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(38400);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  sdfn();

}
void loop() {
  // nothing happens after setup finishes.
}


void sdfn (){
  
  Serial.print("Initializing SD card...");
  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    while (1);
  }

  // create a new file
  char filename[] = "LOGGER00.txt";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! SD.exists(filename)) {
      // only open a new file if it doesn't exist
      myFile = SD.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }


  // open a new file and immediately close it:
  Serial.println("Creating example.txt...");
  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {
    myFile.print("Hooray, it works!");
    Serial.println("Printed text in file");    
    myFile.close();

  }
  else {
    Serial.println("NOOP Not working");
    while(1);
  }

  // Check to see if the file exists:
  if (SD.exists(filename)) {
    Serial.println("example.txt exists.");
  } else {
    Serial.println("example.txt doesn't exist.");
  }

}

//void initializeSD(){
//  // see if the card is present and can be initialized:
//    if (!SD.begin(4)) {
//      Serial.println("Card failed, or not present");
//      err = true;
//      // don't do anything more:
//      while (1);
//    }
//    Serial.println("card initialized.");
//
//    //Create a file with new name
//    if (!loadSDFile()) {
//      Serial.println("Failed to create file");
//      err = true;
//      while(1);
//    }
//    else {
//      Serial.println("File name created!");
//    }
//
//    myFile = SD.open("test.txt", FILE_WRITE);
//    
//    if (myFile) {      
//      //Print Date in File
//      myFile.print("V ");
//      myFile.println(VERSION);
//      
//      myFile.println("Date: ");
//      myFile.print(DAY);
//      myFile.print(MONTH);
//      myFile.println(YEAR);
//  
//      //Print Header Files  - - meters, pascal, est_alt, mpuPitch, mpuRoll, mpuYaw, OutputX, OutputY
//
//      myFile.print("Height");
//      myFile.print(",");
//      myFile.print("Pascal");
//      myFile.print(",");
//      myFile.print("Kalman_Height");
//      myFile.print(",");
//      myFile.print("mpuPitch");
//      myFile.print(",");
//      myFile.print("mpuRoll");
//      myFile.print(",");
//      myFile.print("mpuYaw");
//      myFile.print(",");
//      myFile.print("OutputX");
//      myFile.print(",");
//      myFile.println("OutputY");
// 
//      myFile.close();
//      Serial.println("File Created and File Closed");
//
//    } else {
//      #ifdef SERIAL_DEBUG
//        Serial.println("Error opening file");
//      #endif
//      digitalWrite(RLED, HIGH);
//      while(1);
//    }
//    
//}
