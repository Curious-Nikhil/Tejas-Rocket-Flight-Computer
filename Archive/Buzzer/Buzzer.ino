// BEFORE SETUP
unsigned long previousMillis = 0;
int pauseBetweenNotes;
int noteDuration;
int b_freq;
boolean outputTone = false;                // Records current state

const int buzzer_pin = 8;
void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

}

void loop() {

buzzer(3000, 500, 1000);


}


void buzzer(int b_freq, int noteDuration, int pauseBetweenNotes) {
  //https://arduino.stackexchange.com/a/26069

  unsigned long currentMillis = millis();
  unsigned long previousMillis;
  boolean outputTone;
  const int buzzer_pin;
  
  if (outputTone) {
    // We are currently outputting a tone
    // Check if it's been long enough and turn off if so
    if (currentMillis - previousMillis >= noteDuration) {
      previousMillis = currentMillis;
      noTone(buzzer_pin);
      outputTone = false;
    }

  } else {
    // We are currently in a pause
    // Check if it's been long enough and turn on if so
    if (currentMillis - previousMillis >= pauseBetweenNotes) {
      previousMillis = currentMillis;
      tone(buzzer_pin, b_freq);
      outputTone = true;
    }
  }
}






//
//void status_queue(const unsigned long period, const int b_time, const int b_freq) {
//     unsigned long currentMilllis = millis();
//
//     if (currentMillis - startMillis >= period)
//     {
//      tone(8, b_freq, b_time);
//      Serial.println("inside if");
//      Serial.println(currentMillis - startMillis);
//     }
//
//     unsigned long starMillis = currentMillis;
//}
