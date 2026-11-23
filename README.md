# PongBot — IT & Robotics Labs (ECAM LaSalle, EENG4)

Pong físico con dos brazos Poppy Ergo Jr planares (XL-320), una cámara cenital con ArUco
y joysticks Arduino. **Un solo programa** (`bin/pongbot`) con menú arcade y 5 modos:

| # | Modo | P1 (brazo A) | P2 (brazo B) |
|---|------|--------------|--------------|
| 1 | IA (1 brazo) | IA | — (pared) |
| 2 | JOYSTICK (1 brazo) | joystick | — (pared) |
| 3 | JOYSTICK vs IA | joystick | IA |
| 4 | JOYSTICK vs JOYSTICK | joystick | joystick |
| 5 | IA vs IA | IA | IA |

## Estado (17/11/2026) — probado en la mesa ✅

| Modo | Estado |
|---|---|
| 1. IA (brazo A y brazo B) | ✅ |
| 2. Joystick | ✅ |
| 3. Joystick vs IA | ✅ |
| 4. Joystick vs Joystick | ✅ (2 joysticks en un Arduino) |
| 5. IA vs IA | ✅ |

**Valores medidos en la mesa** (ya están en `config/pongbot.yaml`):

| | Brazo A | Brazo B |
|---|---|---|
| U2D2 (`/dev/serial/by-id/...`) | `FT4TFLHO` | `FT9MIQP7` |
| Hombro fijo (`shoulder_ticks`) | 341 | 231 |
| Límite izquierdo (`lateral_min`, pared) | -6.0 | -5.8 |
| `lateral_center` | 0.0 | -0.3 |
| Bola tocando la pala (`paddle_u`) | 30.8 | 29.9 |
| Disparo del smash (`smash_trigger_u`) | 29.8 | 28.9 |

Smash: retroceso a y=10.5 → golpe hasta y=16.5 (se reduce solo donde no alcanza), 90/150 ms,
anticipación 0.12 s. Paredes laterales en Y=±10.8. Velocidad lateral máx. 60 cm/s.

**Joysticks:** los dos en el mismo Arduino Uno (`arduino/arduino_joystick_dual`):
J1 → A0, A1, D2 (5 V) · J2 → A2, A3, D3 (alimentado a **3.3 V**: centro 337, smash con Y > 560 o botón).
Subir el sketch:
```bash
arduino-cli compile --fqbn arduino:avr:uno arduino/arduino_joystick_dual
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno arduino/arduino_joystick_dual
```

**Limitaciones conocidas**
- Las esquinas de la mesa (|Y| > ~8.5) quedan fuera del alcance de la pala sin chocar con la pared.
  Solución física: pala más ancha o cuñas en las esquinas.
- La muñeca (ID 4) a veces no responde (`There is no status packet!`), sobre todo en movimientos
  rápidos hacia la izquierda. El código reenvía la posición; revisar cables/conectores.
- Si cambia la luz hay que recalibrar el color: `./bin/hsv_tuner -c 2` → `s` → commit de
  `data/color_params_data.xml`.

---

## 1. Compilar

```bash
git pull   # rama main (o integration/final antes de fusionar el PR)
make            # pongbot + herramientas
make test       # autotest sin hardware (debe terminar en "TODO OK")
```
Requisitos: OpenCV 4 (`libopencv-dev`, con `aruco`) y Dynamixel SDK en `/usr/local`
(`-ldxl_x64_cpp`). Si al ejecutar sale `libdxl_x64_cpp.so: cannot open shared object`:
`sudo ldconfig` (o `export LD_LIBRARY_PATH=/usr/local/lib`).

## 2. Ejecutar

```bash
./bin/pongbot                 # check de hardware -> menú -> partida
./bin/pongbot --mode 3        # directo a JOYSTICK vs IA
./bin/pongbot --sim           # SIN hardware: bola y brazos simulados (probar lógica/GUI)
./bin/pongbot --no-motors     # cámara real, los brazos NO se mueven (calibrar visión)
./bin/pongbot --arm B --mode 1   # modos de 1 brazo con el brazo B
```

**Icono y arranque automático** (abre la GUI con el menú al iniciar sesión):
```bash
./scripts/install_autostart.sh            # icono en escritorio/menú + arranque automático
./scripts/install_autostart.sh --no-auto  # solo el icono
./scripts/install_autostart.sh --remove   # quitar todo
```
Los mensajes de cada ejecución se guardan en `logs/`.

**Teclas en partida:** `X` armar/desarmar brazos · `H` home · `C` recalibrar ArUco ·
`V` ver máscara de la bola · `SPACE`/`B` gol manual P1/P2 · `M` menú · `ESC` salir.
**Teclado como joystick** (si un Arduino falla): P1 `A`/`D` + `W` smash · P2 `J`/`L` + `I` smash.

Los goles se cuentan solos cuando la bola pasa la `goal_u` de un brazo (`game.auto_goals`).
Las partidas se guardan en `data/scores.csv`.

## 3. Herramientas

| Comando | Para qué |
|---|---|
| `./bin/arm_test --arm A` (o `B`) | Probar/calibrar un brazo sin cámara: home, barrido lateral, smash, IK, ángulos, ticks. |
| `./bin/joystick_test` | Ver los valores crudos que manda cada Arduino. |
| `./bin/hsv_tuner -c 2` | Ajustar el color de la bola; `S` guarda en `data/color_params_data.xml`. |
| `./bin/MonoCameraCalibration`, `./bin/CameraCalibration` | Calibración intrínseca de la cámara. |
| `arduino/arduino_joystick/` | Sketch del joystick (x,y,botón). |
| `./bin/fan_test` | Probar los ventiladores (derecho / izquierdo). Solo actúan en IA vs IA: ver `docs/fans.md`. |
| `arduino/arduino_pongbot/` | Sketch actual del Arduino: 2 joysticks + ventiladores. |
| `make legacy` | Plan B: los programas antiguos que ya funcionaron (ver `legacy/README.md`). |

## 4. Configuración: `config/pongbot.yaml`

Todo lo que depende del montaje está ahí (**no hace falta recompilar**): índice de cámara,
puertos, IDs de los motores, sentido de giro de cada motor (`dir`), en qué extremo está cada
brazo (`side`), sentido lateral (`lateral_sign`), umbrales del smash, tiempos, etc. Cada campo
está comentado.

Coordenadas: la homografía ArUco da **X** (a lo largo de la mesa) e **Y** (a lo ancho) en cm.
Para cada brazo se usa `u = side * X` (u crece cuando la bola se acerca a ese brazo).
El robot recibe `target_x = lateral_sign * (ball_Y - lateral_center)` y se mueve en su línea
`home_y = 13.5`, igual que el código que funcionó.

---

## 5. ✅ Checklist para la mesa (en este orden)

**0. Antes de tocar nada:** `make && make test` → `TODO OK`. Probar `./bin/pongbot --sim --mode 5`.

**1. Puertos** (`ls /dev/ttyUSB* /dev/ttyACM* /dev/video*`)
- [ ] ¿Cuántos U2D2? Si los dos brazos van en el **mismo** U2D2, poner el mismo `port` y los IDs
      del brazo B distintos (p. ej. `[5,6,7,8]`). Si van en **dos** U2D2: `ttyUSB0` y `ttyUSB1`.
- [ ] Joysticks: uno o dos Arduinos → `joysticks[].device` / `channel`.
- [ ] `./bin/joystick_test` → 3 números `x,y,btn`: centro ~512, el 3º pasa a 0 al pulsar el botón.

**2. Brazo A** (el que ya funcionaba): `./bin/arm_test --arm A`
- [ ] `1` home: pala paralela, en la línea de defensa.
- [ ] `2` barrido: la pala va de izquierda a derecha (vista desde el robot).
- [ ] `4` smash con x = 0, x = 4, x = -4.

**3. Brazo B** ("parecía ir al revés"): `./bin/arm_test --arm B`
- [ ] `1` home: ¿se ve igual que A? Si el codo dobla al otro lado → cambiar signo de `dir[1]`;
      si la pala no queda paralela → `dir[2]`. Si el hombro queda mal, ajustar `shoulder_ticks`
      (si está montado en espejo puede ser `1023 - 341 = 682`).
- [ ] `2` barrido: si va de derecha a izquierda → cambiar signo de `dir[0]`.
- [ ] Medir L1, L2, L3 del brazo B y ponerlos en `links`.
- [ ] `8`: comprobar que `home_y` es alcanzable en todo el rango lateral.

**4. Visión**: `./bin/hsv_tuner -c 2` (solo debe verse la bola, `S` guarda) y luego
`./bin/pongbot --no-motors --mode 5`
- [ ] Se ven los 4 ArUco → "Homografía fijada" (si no, `C` para reintentar).
- [ ] Mover la bola hacia el brazo A: en la vista cenital `ball X` debe **crecer** (si decrece:
      `vision.flip_x: 1`). Hacia el B debe decrecer.
- [ ] Poner la bola tocando la pala de A en home → anotar `ball X` → `paddle_u` de A.
      Igual con B (con el signo cambiado: `paddle_u = -X`).
- [ ] Bola detrás de cada pala → anotar X → `goal_u` (un poco más que `paddle_u`).
- [ ] El indicador de la pala en la vista cenital sigue la bola por el lado correcto. Si el
      brazo B se va al lado contrario → cambiar su `lateral_sign`.

**5. Con motores**, uno a uno:
- [ ] `./bin/pongbot --mode 1` (A con IA). `X` desarma si algo va mal.
- [ ] `./bin/pongbot --mode 1 --arm B`.
- [ ] `--mode 2` (joystick A), `--mode 3`, `--mode 4`, `--mode 5`.

**Ajuste fino del smash** (todo en el YAML):
- Golpea **tarde** → bajar `smash_trigger_u` o subir `smash_lead_s`.
- Golpea **pronto** (la bola aún no llega) → al revés.
- Poca fuerza → subir `strike_ms` / bajar `recoil_ms`; el `strike_y` efectivo se reduce solo
  si no es alcanzable en esa x (en los extremos ≈ 14 cm).
- El smash se dispara aunque la bola pase lejos → bajar `smash_lateral_window`.

---

## 6. Arquitectura

```
config/pongbot.yaml        parámetros de la mesa
include/pongbot/, src/pongbot/
  config      lee el YAML
  arm         PongArm: IK de Valeria (sin cambios) + smash NO bloqueante + bus compartido
  vision      CameraThread (hilo propio) + ArUco/homografía + bola HSV/circularidad + velocidad
  controllers AIController (sigue/predice + smash) · JoystickController (Arduino o teclado)
  joystick    lector serie no bloqueante ("x,y" / "x1,y1,x2,y2")
  sim         mesa simulada para --sim y para los tests
  game_gui    GUI arcade (check, menú, HUD, vista cenital) + score_manager
  main.cpp    bucle principal: percepción -> control -> brazos -> goles -> dibujo
tools/      arm_test, joystick_test, hsv_tuner, calibración de cámara
tests/      selftest (make test)
arduino/    sketch del joystick (Valeria)
legacy/     código antiguo (plan B)
```

**Qué cambió respecto al código anterior**
- La cámara corre en su propio hilo y nada usa `usleep`: el smash es una máquina de estados
  (RECOIL → STRIKE → RETURN → COOLDOWN) que avanza en cada vuelta del bucle.
- El smash se dispara al **cruzar** `smash_trigger_u` o cuando la bola llegará antes de
  `smash_lead_s` (antes una ventana fija de 22.5–24 cm que a 30 FPS se saltaba).
- El golpe nunca pide un punto fuera del workspace (`maxReachableY`).
- La IA puede predecir dónde llegará la bola, con rebotes en las paredes (`vision.predict`).
- Solo se escriben los motores cuyo valor cambió (menos tráfico en el bus).
- Dos brazos pueden compartir U2D2 (IDs distintos) o ir en puertos distintos.
