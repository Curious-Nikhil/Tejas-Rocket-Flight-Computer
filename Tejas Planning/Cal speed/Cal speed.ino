#include<TimerOne.h>
#define switch1 2
#define switch2 3
int deg=45;
volatile long first_time=0;
volatile long second_time=0;;/'
long elapsed_time;++,,,,,,,,/.
float speed;
float height;
float range;


void setup() {
pinMode(switch1,INPUT);
pinMode(switch2,INPUT);
Serial.begin(9600);
Serial.println("waiting for projectile");
Serial.println("   Speed(Mps):\tHeight    tRange:");

Timer1.initialize(500000);                        
attachInterrupt(digitalPinToInterrupt(2),first_Interrupt,HIGH);
attachInterrupt(digitalPinToInterrupt(3),second_Interrupt,LOW);
Timer1.attachInterrupt( isr_timer );

}

void loop() {

elapsed_time=second_time-first_time;      //elapses_time being calculated in millis 
speed = (0.9*pow(10,3))/elapsed_time;   //0.9 m is the distance between two switch
height = (pow(speed,2)*sin(deg*0.017))/19.6; //0.017 is the value when degree is converted into radian and 19.6 is two times gravity
range =  (pow(speed,2)*sin(deg*0.034))/9.8;
Serial.print("\t");
Serial.print(speed); 
Serial.print("\t");
Serial.print(height);
 Serial.print("\t");
Serial.print(range);
 Serial.println();  
delay(10000);

}

void isr_timer()
{
  
}
void first_Interrupt()
{
  first_time=millis();
}

void second_Interrupt()
{
  second_time=millis();
  
}


