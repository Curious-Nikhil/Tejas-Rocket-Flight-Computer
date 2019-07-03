
unsigned long startMillis;  //some global variables available anywhere in the program
unsigned long currentMillis;
unsigned long counttime;
const unsigned long period = 100;  //the value is a number of millisecondsunsigned long startMillis2;  //some global variables available anywhere in the program
const byte ledPin = 6;    //using the built in LED



void setup()
{
  Serial.begin(115200);  //start Serial in case we need to print debugging info
  pinMode(ledPin, OUTPUT);
  startMillis = millis();  //initial start time
}

void loop()
{
 RED();

 Serial.print(counttime/1000);
 Serial.println(" Seconds Have Passed!");
}


void RED() {

    currentMillis = millis();  //get the current "time" (actually the number of milliseconds since the program started)
  if (currentMillis - startMillis >= period)  //test whether the period has elapsed
  {
    //Serial.println(currentMillis);
    counttime = currentMillis;
    digitalWrite(ledPin, !digitalRead(ledPin));  //if so, change the state of the LED.  Uses a neat trick to change the state
    startMillis = currentMillis;  //IMPORTANT to save the start time of the current LED state.
  }
}
