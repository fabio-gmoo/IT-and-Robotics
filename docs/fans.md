# Ventiladores — guía de montaje

Ventilador: **Foxconn PV902512PSPF (HP 432768-001)**, 92 mm, **12 V · 0.40 A**, 4 cables.
Montaje actual: **2 ventiladores, uno detrás de cada brazo**, soplando a lo largo de la mesa.

## Qué hacen
Si la bola se queda **trancada en la zona central** (se mueve menos de 1.5 cm en 1.2 s y está
fuera del alcance de los brazos, |X| ≤ 26 cm), se enciende el ventilador **del lado donde está**:

| Bola en… | Se enciende | Sopla hacia |
|---|---|---|
| mitad del brazo **A** (X > 0, derecha en la GUI) | ventilador detrás de **A** (pin **D5**) | el brazo B |
| mitad del brazo **B** (X < 0, izquierda en la GUI) | ventilador detrás de **B** (pin **D10**) | el brazo A |

Mínimo 0.8 s encendido, se apaga cuando la bola vuelve a moverse, máximo 3 s seguidos (luego
descansa 1.5 s). Nunca los dos a la vez. Tecla **F** en partida = activar/desactivar.
En la vista cenital: círculo amarillo **FAN** = encendido, y `STUCK` junto a la posición de la bola.

## Conexión (cada ventilador)

| Cable | Va a |
|---|---|
| **Amarillo** (+12 V) | **+12 V** de la fuente |
| **Negro** (GND) | **GND** de la fuente |
| **Azul** (PWM) | **D5** (ventilador de A) / **D10** (ventilador de B) |
| **Verde** (tacómetro) | sin conectar (aislar) |
| **GND fuente ↔ GND Arduino** | **un cable, obligatorio** (referencia de la señal PWM) |

⚠️ Nunca conectéis los +12 V a un pin del Arduino.

**Limitación:** por el cable azul estos ventiladores **no se paran del todo**: "apagado" = velocidad
mínima, "encendido" = máxima (es así por diseño del ventilador). Para apagarlos del todo hace falta
cortar la alimentación con algo que maneje el Arduino (MOSFET logic-level, relé, driver L298N o
placa ULN2003): el pin D5/D10 iría a ese componente en vez de al cable azul. El código no cambia
(con relés activos en LOW: `FAN_ACTIVE_LOW = true` en el sketch).

## Subir el sketch (joysticks + ventiladores)

```bash
cd ~/PongBot_final && git pull && make
arduino-cli compile --fqbn arduino:avr:uno arduino/arduino_pongbot
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/arduino_pongbot
```
(cerrad antes `pongbot` / `joystick_test`)

## Probar

```bash
./bin/fan_test        # ventilador de A 3 s, luego el de B 3 s
./bin/fan_test A      # solo el de A, 5 s
./bin/fan_test B      # solo el de B, 5 s
./bin/pongbot --mode 5
```
En el juego: dejad la bola quieta en el centro, un poco hacia el lado de A → a los ~1.2 s sopla el
ventilador de A. Igual hacia el lado de B.

## Ajustes (`config/pongbot.yaml`, sección `fans`)

| Parámetro | Valor | Qué hace |
|---|---|---|
| `fan_pos_x` / `fan_neg_x` | 0 / 3 | qué pin es cada ventilador (0=D5, 1=D6, 2=D9, 3=D10) |
| `middle_x` | 26 cm | zona central; más allá actúa el brazo con el smash |
| `stuck_radius` / `stuck_time_s` | 1.5 cm / 1.2 s | cuándo se considera trancada |
| `min_on_s` / `max_on_s` / `rest_s` | 0.8 / 3 / 1.5 s | tiempos de encendido |

Si al probar sale al revés (sopla el de A cuando la bola está del lado de B), intercambiad los
valores de `fan_pos_x` y `fan_neg_x`.
