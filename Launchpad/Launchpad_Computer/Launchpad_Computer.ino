#define mos 2
#define pushbutton A0
#define buzzer 7
#define LED 12

int buttonState = 0;

void setup() {

  pinMode(mos, OUTPUT);
  pinMode(pushbutton, INPUT);

  Serial.begin(9600);

  //Startup Sound
  tone(buzzer, 2000, 300);
  delay(500);
  tone(buzzer, 3000, 300);
  tone(buzzer, 3500, 300);
  delay(500);
  tone(buzzer, 2500, 300);

}

void loop() {

  buttonState = digitalRead(pushbutton);
  digitalWrite(mos, LOW); //Set Mos pin LOW, dont launch by accident!!

  Serial.println(buttonState);

  digitalWrite(LED, LOW);

  if (buttonState == HIGH) {

    tone(buzzer, 500, 500);
    digitalWrite(LED, HIGH);

    Serial.println("Starting a 10 second Timer!");
    delay(1000);
    digitalWrite(LED, LOW);


    tone(buzzer, 3500, 500);
    Serial.println("9");
    delay(1000);
    digitalWrite(LED, HIGH);


    tone(buzzer, 3500, 500);
    Serial.println("8");
    delay(1000);
    digitalWrite(LED, LOW);


    tone(buzzer, 3500, 500);
    Serial.println("7");
    delay(1000);
    digitalWrite(LED, LOW);


    tone(buzzer, 3500, 500);
    Serial.println("6");
    delay(1000);
    digitalWrite(LED, LOW);


    tone(buzzer, 3500, 500);

    Serial.println("5");
    delay(1000);
    digitalWrite(LED, HIGH);


    tone(buzzer, 3500, 500);
    Serial.println("4");
    delay(1000);
    digitalWrite(LED, LOW);

    tone(buzzer, 3500, 500);
    Serial.println("3");
    delay(1000);
    digitalWrite(LED, HIGH);

    tone(buzzer, 3500, 500);
    Serial.println("2");
    delay(1000);
    digitalWrite(LED, LOW);


    tone(buzzer, 3500, 500);

    Serial.println("1");
    delay(1000);
    digitalWrite(LED, HIGH);

    tone(buzzer, 3500, 500);
    Serial.println("0");
    delay(1000);
    digitalWrite(LED, LOW);

    tone(buzzer, 4500, 2000);
    Serial.println("Launch OFF!!");
    digitalWrite(LED, HIGH);

    Serial.println("ON");
    digitalWrite(mos, HIGH);
    delay(3000);

    tone(buzzer, 2000, 300);
    delay(500);
    tone(buzzer, 3000, 300);
    tone(buzzer, 3500, 300);
    delay(500);
    tone(buzzer, 2500, 300);

  }

  buttonState = 0; //Wait for switc to again. RESET

  Serial.println("OFF");
  digitalWrite(mos, LOW);
}


