// Sketch REAL del joystick (de Valeria). Manda "x,y,btn" a 115200; btn = 0 al pulsar (pin 2).
// PongBot hace smash con el botón o empujando el stick (Y > smash_threshold).
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
