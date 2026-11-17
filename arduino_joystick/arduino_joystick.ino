void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
}

void loop() {
  int x = analogRead(A0);
  int y = analogRead(A1);
  int btn = digitalRead(2);

  Serial.print(x);
  Serial.print(",");
  Serial.print(y);
  Serial.print(",");
  Serial.println(btn);

  delay(20); // 50 Hz
}
