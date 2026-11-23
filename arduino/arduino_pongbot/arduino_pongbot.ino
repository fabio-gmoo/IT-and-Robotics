// =============================================================================
//  PongBot - Arduino Uno: 2 joysticks + 4 ventiladores de esquina
//
//  PC <- Arduino : "x1,y1,b1,x2,y2,b2\n" cada 20 ms (igual que arduino_joystick_dual)
//  PC -> Arduino : "F<mask>\n"  mask = 0..15, bit i = ventilador i encendido
//                  (lo manda bin/pongbot y bin/fan_test)
//
//  Joysticks:  J1 -> A0, A1, D2 (5 V)     J2 -> A2, A3, D3 (3.3 V)
//  Ventiladores (puerta de cada MOSFET, a través de 220 ohm):
//     FAN 0 -> D5   esquina +X +Y  (lado brazo A, ARRIBA en la vista cenital)
//     FAN 1 -> D6   esquina +X -Y  (lado brazo A, ABAJO)
//     FAN 2 -> D9   esquina -X +Y  (lado brazo B, ARRIBA)
//     FAN 3 -> D10  esquina -X -Y  (lado brazo B, ABAJO)
//
//  SEGURIDAD: si el PC deja de mandar órdenes durante 1.5 s (programa cerrado,
//  cable suelto...) se apagan todos los ventiladores.
// =============================================================================

const int FAN_PINS[4] = {5, 6, 9, 10};

// MOSFET (o módulo MOSFET): HIGH = encendido  -> false
// Módulo de RELÉS típico (se activa con LOW)  -> true
const bool FAN_ACTIVE_LOW = false;

const unsigned long WATCHDOG_MS = 1500;
const unsigned long SEND_PERIOD_MS = 20;

unsigned long lastCommandMs = 0;
unsigned long lastSendMs = 0;
int fanMask = 0;
char rxBuf[16];
int rxLen = 0;

void applyFans(int mask) {
  for (int i = 0; i < 4; i++) {
    bool on = (mask >> i) & 1;
    digitalWrite(FAN_PINS[i], (on != FAN_ACTIVE_LOW) ? HIGH : LOW);
  }
  fanMask = mask;
}

void handleLine(const char* line) {
  // "F<0..15>"
  if (line[0] == 'F' || line[0] == 'f') {
    int mask = atoi(line + 1);
    if (mask >= 0 && mask <= 15) {
      applyFans(mask);
      lastCommandMs = millis();
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(2, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  for (int i = 0; i < 4; i++) pinMode(FAN_PINS[i], OUTPUT);
  applyFans(0);
}

void loop() {
  // 1) Órdenes del PC (no bloqueante)
  while (Serial.available() > 0) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (rxLen > 0) {
        rxBuf[rxLen] = '\0';
        handleLine(rxBuf);
        rxLen = 0;
      }
    } else if (rxLen < (int)sizeof(rxBuf) - 1) {
      rxBuf[rxLen++] = c;
    } else {
      rxLen = 0;  // línea demasiado larga: descartar
    }
  }

  // 2) Watchdog: sin órdenes => todo apagado
  if (fanMask != 0 && millis() - lastCommandMs > WATCHDOG_MS) applyFans(0);

  // 3) Joysticks (cada 20 ms, sin delay() para no perder órdenes)
  if (millis() - lastSendMs >= SEND_PERIOD_MS) {
    lastSendMs = millis();
    Serial.print(analogRead(A0)); Serial.print(',');
    Serial.print(analogRead(A1)); Serial.print(',');
    Serial.print(digitalRead(2)); Serial.print(',');
    Serial.print(analogRead(A2)); Serial.print(',');
    Serial.print(analogRead(A3)); Serial.print(',');
    Serial.println(digitalRead(3));
  }
}
