# Ventiladores

Dos ventiladores Foxconn PV902512PSPF (12 V · 0.40 A), uno **detrás de cada brazo**.
**Solo funcionan en el modo 5 (IA vs IA).** En los modos con joystick no se activan.

## Regla
| Bola | Ventilador |
|---|---|
| lenta (< `slow_speed`, 12 cm/s) yendo a la **izquierda** | **derecho** (D5) → llega con más velocidad a la pala izquierda |
| lenta yendo a la **derecha** | **izquierdo** (D10) |
| casi parada (< 2 cm/s a lo largo) | el de su mitad, que la saca al otro lado |
| rápida / sin bola | ninguno |

La velocidad la calcula la visión (flecha de la vista cenital); la GUI muestra
`Fans: left - right ON | ball 7 cm/s`.

## Conexión
| Cable del ventilador | Va a |
|---|---|
| Amarillo (+12 V) | +12 V de la fuente |
| Negro (GND) | GND de la fuente |
| Azul (PWM) | **D5** (derecho, detrás del brazo A) / **D10** (izquierdo, detrás del brazo B) |
| Verde (tacómetro) | sin conectar |
| **GND fuente ↔ GND Arduino** | obligatorio |

Por el cable azul estos ventiladores no se paran del todo: "apagado" = velocidad mínima.

## Sketch y pruebas
```bash
arduino-cli compile --fqbn arduino:avr:uno arduino/arduino_pongbot
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/arduino_pongbot
./bin/fan_test          # derecho 3 s, izquierdo 3 s   (R / L para uno solo)
./bin/pongbot --mode 5
```
El sketch manda los joysticks (`x1,y1,b1,x2,y2,b2`) y recibe `F<mask>`; si no recibe nada en
1.5 s pone los ventiladores al mínimo (seguridad). El PC vacía siempre lo que manda el Arduino:
si no, el buffer USB se llena y las órdenes dejan de llegar.

Ajustes en `config/pongbot.yaml`, sección `fans`: `slow_speed`, `stopped_speed`, `left_fan`, `right_fan`.
