// Dos joysticks en UN Arduino Uno.
// Manda una línea "x1,y1,b1,x2,y2,b2" a 115200 baudios (~50 Hz).
// b = 0 cuando el botón está pulsado (INPUT_PULLUP).
//
// Cableado:
//   Joystick 1 (P1, brazo A): VRx -> A0, VRy -> A1, SW -> D2
//   Joystick 2 (P2, brazo B): VRx -> A2, VRy -> A3, SW -> D3
//   Los dos: +5V -> 5V, GND -> GND
//
// En config/pongbot.yaml: P1 y P2 con el mismo device (/dev/ttyACM0),
// P1 channel 0, P2 channel 1.

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
}

void loop() {
  int x1 = analogRead(A0);
  int y1 = analogRead(A1);
  int b1 = digitalRead(2);
  int x2 = analogRead(A2);
  int y2 = analogRead(A3);
  int b2 = digitalRead(3);

  Serial.print(x1); Serial.print(',');
  Serial.print(y1); Serial.print(',');
  Serial.print(b1); Serial.print(',');
  Serial.print(x2); Serial.print(',');
  Serial.print(y2); Serial.print(',');
  Serial.println(b2);

  delay(20);  // 50 Hz
}
