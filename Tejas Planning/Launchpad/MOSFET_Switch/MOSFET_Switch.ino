#define mos 2


void setup() {

  pinMode(mos, OUTPUT);
  Serial.begin(9600);


}

void loop() {

  Serial.println("ON");
  digitalWrite(mos, HIGH);
  delay(1000);

  Serial.println("OFF");
  digitalWrite(mos, LOW);
}
