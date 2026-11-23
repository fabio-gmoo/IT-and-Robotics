# Ventiladores de esquina — guía de montaje

Ventilador: **Foxconn PV902512PSPF (HP 432768-001)**, 92 mm, **12 V · 0.40 A**, 4 cables (PWM).
Objetivo: cuando la bola se queda en una esquina, se enciende **solo** el ventilador de esa
esquina y la empuja de vuelta al juego.

## 1. Material

| Por cada ventilador (×4) | Común |
|---|---|
| 1 MOSFET N **logic-level** (IRLZ44N, IRL540N…) **o** 1 módulo MOSFET (IRF520) | Fuente **12 V ≥ 1 A** (2 A si se prueban los 4 a la vez) |
| 1 resistencia **220 Ω** (puerta) | Protoboard / regleta |
| 1 resistencia **10 kΩ** (puerta → GND) | Cables |
| 1 diodo **1N4007** (o 1N4148/1N5819) | |

> Un módulo de **4 relés** también sirve (más simple, hace "clic"): ver §5.

## 2. Cables del ventilador

El conector de 4 pines PWM estándar (el pin 1 suele estar marcado con un triángulo ▲):

| Pin | Señal | Color habitual | Conexión |
|---|---|---|---|
| 1 | GND | **negro** | → **Drain** del MOSFET |
| 2 | +12 V | **amarillo** (a veces rojo) | → **+12 V** de la fuente |
| 3 | Tacómetro | verde (a veces blanco/amarillo) | **sin conectar** (aislar) |
| 4 | PWM | **azul** | **sin conectar** (aislar) |

⚠️ HP a veces usa colores distintos. **Fiaos de la posición en el conector**, o medid con el
multímetro. Si hay rojo **y** amarillo: rojo = +12 V y amarillo = tacómetro.
El cable PWM no sirve para apagarlo: con PWM al 0 % estos ventiladores siguen girando despacio.
Por eso se corta la **alimentación** con el MOSFET.

## 3. Esquema (uno por ventilador)

```
   +12 V fuente ───────────────┬───────────────► rojo/amarillo (+12 V ventilador)
                               │
                           1N4007  (banda/cátodo hacia +12 V)
                               │
                               ├───────────────► negro (GND ventilador)
                               │
                            Drain
   Arduino Dx ──[220 Ω]──── Gate      MOSFET N (IRLZ44N: G-D-S mirando el frente)
                     │      Source
                  [10 kΩ]      │
                     │         │
   GND Arduino ──────┴─────────┴───────────────── GND fuente 12 V   ◄── ¡GND COMÚN!
```

- **GND común**: el GND de la fuente de 12 V **tiene que** ir al GND del Arduino.
- **Nunca** conectéis los 12 V a ningún pin del Arduino.
- El diodo protege el MOSFET del pico al apagar el motor (cátodo = banda → +12 V).

## 4. Pines y esquinas

| Ventilador | Pin Arduino | Esquina (coordenadas de mesa) | En la vista cenital de la GUI |
|---|---|---|---|
| FAN 0 | **D5** | +X +Y | lado brazo **A**, **arriba** |
| FAN 1 | **D6** | +X −Y | lado brazo **A**, **abajo** |
| FAN 2 | **D9** | −X +Y | lado brazo **B**, **arriba** |
| FAN 3 | **D10** | −X −Y | lado brazo **B**, **abajo** |

Los joysticks siguen igual (A0, A1, D2 / A2, A3, D3). Orientad cada ventilador **en diagonal
hacia el centro de la mesa**, un poco inclinado hacia abajo para que el aire "barra" la mesa.

## 5. Alternativa: módulo de 4 relés

`IN1..IN4` → D5, D6, D9, D10 · `VCC` → 5 V · `GND` → GND.
En cada relé: +12 V de la fuente → `COM`, `NO` → rojo/amarillo del ventilador, negro → GND fuente.
Casi todos los módulos se activan con LOW: en el sketch poned `FAN_ACTIVE_LOW = true`.

## 6. Subir el sketch (sustituye al de los joysticks, que sigue funcionando igual)

```bash
cd ~/PongBot_final && git pull && make
arduino-cli compile --fqbn arduino:avr:uno arduino/arduino_pongbot
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/arduino_pongbot
```
(cerrad antes `pongbot` / `joystick_test`, si no el puerto está ocupado)

## 7. Probar

```bash
./bin/joystick_test    # los joysticks deben seguir dando 6 números
./bin/fan_test         # enciende FAN 0, 1, 2, 3 uno a uno (3 s cada uno)
./bin/fan_test 2       # solo el FAN 2 durante 5 s
./bin/fan_test all     # los 4 a la vez (prueba de la fuente)
```
Comprobad que cada ventilador está en **la esquina que dice la tabla**. Si no, intercambiad los
cables de los pines (no hace falta tocar el código).

Luego `./bin/pongbot --mode 5`: dejad la bola en una esquina → a los 0.4 s se enciende ese
ventilador (círculo amarillo "FAN" en la vista cenital). Tecla **F** = activar/desactivar.

## 8. Ajustes (`config/pongbot.yaml`, sección `fans`)

| Parámetro | Valor | Qué hace |
|---|---|---|
| `corner_x`, `corner_y` | 24, 7.5 cm | tamaño de la zona de esquina |
| `on_delay_s` | 0.4 s | tiempo en la esquina antes de encender (no salta si la bola solo pasa) |
| `hold_s` | 0.6 s | sigue encendido tras salir |
| `max_on_s` / `rest_s` | 4 s / 1 s | máximo seguido y descanso |

Seguridad: si `pongbot` se cierra o se cuelga, el Arduino apaga todos los ventiladores a los
1.5 s (watchdog). Al volver al menú también se apagan.
