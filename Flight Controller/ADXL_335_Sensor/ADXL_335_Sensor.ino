#define ADC_ref 2.56
#define zero_x 1.569
#define zero_y 1.569
#define zero_z 1.569
#define sen_x 0.3
#define sen_y 0.3 
#define sen_z 0.3

const int xpin = A0;                 // x-axis of the accelerometer
const int ypin = A1;                  // y-axis
const int zpin = A2;                  // z-axis (only on 3-axis models)
const int LED = 13;

float val_x;
float val_y;
float val_z;

float xv;
float yv;
float zv;
float angle_x;
float angle_y;
float angle_z;

void setup() {
  // initialize the serial communications
  Serial.begin(9600);
  pinMode(LED, OUTPUT);
}

void loop() {
  val_x = analogRead(xpin);
  val_y = analogRead(ypin);
  val_z = analogRead(zpin);

  xv = (val_x/1024*ADC_ref - zero_x)/sen_x;
  yv = (val_z/1024*ADC_ref - zero_y)/sen_y;
  angle_z = atan2(-yv, -xv)*57.2957795+180;
  
  Serial.print(xv);
  Serial.print("\t");
  
  Serial.print(angle_z);
  Serial.print("\n");
  delay(10);

}


//  
//  // print the sensor values:
//  Serial.print(analogRead(xpin));
//  // print a tab between values:
//  Serial.print("\t");
//  Serial.print(analogRead(ypin));
//  // print a tab between values:
//  Serial.print("\t");
//  Serial.print(analogRead(zpin));
//  Serial.println();
//  // delay before next reading:
//  delay(100);


