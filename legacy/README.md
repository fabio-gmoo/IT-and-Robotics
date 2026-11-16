# legacy/ — código antiguo (PLAN B)

Nada de esta carpeta se usa en `bin/pongbot`. Se guarda tal cual estaba para no perder
trabajo y como **plan B** si algo del código nuevo falla en la mesa.

## `pong_robot_valeria/` — lo que SÍ movió el brazo (20/10)
| Archivo | Origen | Qué es |
|---|---|---|
| `pong_robot.hpp` | rama `feature/kinematics-dynamixel-control` | Clase `PongRobot` (IK 3 eslabones, hombro fijo 341). La IK de `src/pongbot/arm.cpp` es una copia literal (lo comprueba `make test`). |
| `RobotServoing_autonomous.cpp` | misma rama | Portero autónomo con ArUco + smash por ventana fija (cámara índice 2). |
| `joystick_teleop.cpp` | `develop` | Teleoperación con Arduino `/dev/ttyACM0`, smash con `usleep`. |
| `interactive_test.cpp`, `main_robot_test.cpp` | `develop` | Pruebas manuales del brazo. |

Compilar el plan B:
```bash
make legacy
./bin/legacy_autonomous        # M = armar, ESC = salir
./bin/legacy_joystick_teleop
./bin/legacy_interactive_test
```

## `two_link_stack/` — versión de 2 eslabones (no usada)
`Kinematics.cpp` + `DynamixelHandler.cpp` (L1=5, L2=6, pala a -90°), `RobotServoing.cpp` y
`ArmDetection.cpp` (rama `feature/visionTestBastien`, detección del extremo del brazo por color),
`RedBallTracking.cpp`, `ChessboardOrientationTracking.cpp`. Útil como referencia para el informe.

## Otros
`test_gui.cpp`, `test_score.cpp`: prototipos de la GUI ya integrados en `src/pongbot/`.
