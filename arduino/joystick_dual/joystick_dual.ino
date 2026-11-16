// Sketch de REFERENCIA para el joystick (por si hace falta reflashear).
// Formato que espera PongBot (115200 baudios, una línea por lectura):
//   1 joystick :  "x,y\n"           (A0 = X, A1 = Y)
//   2 joysticks:  "x1,y1,x2,y2\n"   (A0, A1, A2, A3)
// Valores 0..1023, centro ~512. Empujar el stick "hacia delante" debe dar Y > 800
// (si sale al revés, usa Y = 1023 - Y aquí o cambia el cableado).
// En config/pongbot.yaml: con 2 sticks en este Arduino, P1 channel 0 y P2 channel 1
// con el MISMO device.

const bool TWO_STICKS = false;   // true si hay dos joysticks en este Arduino

void setup() {
  Serial.begin(115200);
}

void loop() {
  Serial.print(analogRead(A0));
  Serial.print(',');
  Serial.print(analogRead(A1));
  if (TWO_STICKS) {
    Serial.print(',');
    Serial.print(analogRead(A2));
    Serial.print(',');
    Serial.print(analogRead(A3));
  }
  Serial.print('\n');
  delay(10);  // ~100 Hz
}
