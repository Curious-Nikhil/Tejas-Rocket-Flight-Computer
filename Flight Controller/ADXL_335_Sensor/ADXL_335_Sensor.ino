// these constants describe the pins. They won't change:
const int xpin = A0;                 // x-axis of the accelerometer
const int ypin = A1;                  // y-axis
const int zpin = A2;                  // z-axis (only on 3-axis models)
const int LED = 13;

void setup() {
  // initialize the serial communications:
  Serial.begin(9600);
  pinMode(LED, OUTPUT);

  // Provide ground and power by using the analog inputs as normal digital pins.
  // This makes it possible to directly connect the breakout board to the
  // Arduino. If you use the normal 5V and GND pins on the Arduino,
  // you can remove these lines.
//  pinMode(groundpin, OUTPUT);
//  pinMode(powerpin, OUTPUT);
//  digitalWrite(groundpin, LOW);
//  digitalWrite(powerpin, HIGH);
}

void loop() {
 
  if (analogRead(zpin) >450) {
      digitalWrite(LED, HIGH);
  delay(1000);
  }else{
      digitalWrite(LED, LOW);
  delay(1000);
  }

  
  
  // print the sensor values:
  Serial.print(analogRead(xpin));
  // print a tab between values:
  Serial.print("\t");
  Serial.print(analogRead(ypin));
  // print a tab between values:
  Serial.print("\t");
  Serial.print(analogRead(zpin));
  Serial.println();
  // delay before next reading:
  delay(100);
}

