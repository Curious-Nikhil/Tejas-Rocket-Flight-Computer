//@Nikhil MIshra

unsigned long previousMillis;
unsigned long startMillis;
unsigned long currentMillis;
unsigned long time_interval = 1000;
int counter = 0;

#define mos 2
#define pushbutton 5
#define buzzer 7
#define LED 9 // Red Light
#define GLED 8

void setup()
{
  Serial.begin(9600);

  pinMode(LED, OUTPUT);
  pinMode(GLED, OUTPUT);
}

void loop()
{
  unsigned long time_interval = 1000;
  int i;

  currentMillis = millis();

  if (currentMillis - previousMillis >= (time_interval))
  {
    previousMillis = currentMillis;
    counter++;

    //Launch Sequence Starts
    if (counter == 0)
    {
      Serial.println("Starting a 10 second Timer!");
      Serial.println(10 - counter);
      tone(buzzer, 2500, 500);
      digitalWrite(LED, HIGH);
    }
    else if (counter % 2 == 0)
    {
      //If even
      digitalWrite(LED, HIGH);
      Serial.println(10 - counter);
      tone(buzzer, 3500, 500);
    }
    else
    {
      digitalWrite(LED, LOW);
      Serial.println(10 - counter);
      tone(buzzer, 3000, 500);
    }
  }

    if (counter >= 10)
    {
      Serial.println("LAUNCH OFF");
      tone(buzzer, 4500, 2000);
      digitalWrite(LED, HIGH);
      while (1);
    }
}