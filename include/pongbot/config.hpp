// =============================================================================
//  PongBot - configuración central
//  Todo lo que depende de la mesa / del montaje se lee de config/pongbot.yaml,
//  así se puede ajustar en el laboratorio SIN recompilar.
// =============================================================================
#ifndef PONGBOT_CONFIG_HPP
#define PONGBOT_CONFIG_HPP

#include <array>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

namespace pongbot {

// ---------------------------------------------------------------------------
// Un brazo (robot Poppy Ergo Jr planar: base, hombro fijo, codo, muñeca/pala)
// ---------------------------------------------------------------------------
struct ArmConfig {
    std::string name = "A";
    bool enabled = true;

    // --- Bus Dynamixel ---
    std::string port = "/dev/ttyUSB0";
    int baudrate = 1000000;
    // IDs: base (q1), hombro FIJO, codo (q2), muñeca/pala (q3)
    int id_base = 1, id_shoulder = 2, id_elbow = 3, id_wrist = 4;
    int shoulder_ticks = 341;              // hombro bloqueado en esta posición

    // --- Conversión ángulo -> ticks (XL-320: 0..1023 = 300°, 512 = 0 rad) ---
    // ticks = 512 + dir * deg * 1023/300 + offset
    // Convención de Valeria (la que funcionó): base +1, codo -1, muñeca -1
    std::array<int, 3> dir = {1, -1, -1};      // base, codo, muñeca
    std::array<int, 3> offset = {0, 0, 0};     // corrección de cero (ticks)
    std::array<int, 3> tick_min = {0, 0, 0};   // límites físicos de seguridad
    std::array<int, 3> tick_max = {1023, 1023, 1023};

    // --- Geometría (cm) ---
    double L1 = 5.0, L2 = 5.5, L3 = 6.3;
    double gamma = 0.0;                    // orientación de la pala (rad); 0 = paralela

    // --- Colocación en la mesa ---
    // side = +1: el robot está en el extremo +X de la mesa (la bola se acerca
    //            cuando X aumenta).  side = -1: extremo -X.
    int side = 1;
    // target_x(robot) = lateral_sign * (ball_Y - lateral_center)
    // Brazo A probado: lateral_sign = +1.  Un brazo montado "mirando al otro
    // lado" normalmente necesita -1 (por eso parecía que iba "al revés").
    int lateral_sign = 1;
    double lateral_center = 0.0;           // Y de mesa frente a la base del robot
    double lateral_limit = 7.0;            // |target_x| máximo (cm)
    // Límites asimétricos (p.ej. una pared a la izquierda). Por defecto ±lateral_limit.
    double lateral_min = -7.0;             // x mínima (izquierda del robot)
    double lateral_max = 7.0;              // x máxima (derecha del robot)

    // --- Posiciones en Y del robot (cm, hacia delante desde la base) ---
    double home_y = 13.5;                  // línea defensiva
    double recoil_y = 11.5;                // retroceso antes del golpe
    double strike_y = 15.3;                // golpe (se reduce solo si no alcanza)

    // --- Smash no bloqueante (ms) ---
    int recoil_ms = 60;
    int strike_ms = 120;
    int return_ms = 80;
    int cooldown_ms = 250;

    // --- Disparo del smash (coordenada u = side * X_mesa, cm) ---
    double paddle_u = 21.0;                // u de la pala en home (MEDIR en la mesa)
    double smash_trigger_u = 20.0;         // umbral de cruce (antes: ball_X >= 20)
    double smash_min_speed = 4.5;          // cm/s hacia el robot (antes 0.15 cm/frame)
    double smash_lead_s = 0.12;            // anticipación por latencia
    double smash_lateral_window = 4.0;     // |ball_Y - pala| máx para golpear (cm)
    double goal_u = 23.5;                  // si u > goal_u => gol en contra de este brazo

    // --- Bus ---
    int min_send_ms = 8;                   // período mínimo entre escrituras
    double max_lateral_speed = 60.0;       // cm/s: limita saltos bruscos (0 = sin límite)
    int write_gap_us = 1000;               // pausa entre motores (el viejo usaba 3000)
};

// ---------------------------------------------------------------------------
// Joystick por Arduino (línea serie "x,y,btn\n", "x,y\n", ... a 115200)
// ---------------------------------------------------------------------------
struct JoystickConfig {
    std::string device = "/dev/ttyACM0";
    int channel = 0;            // 0 -> valores 1-2 de la línea, 1 -> valores 3-4
    int center = 512;
    int deadzone_low = 450;
    int deadzone_high = 570;
    int smash_threshold = 800;  // eje Y por encima => smash (0 = solo botón)
    bool invert_x = false;
    bool invert_y = false;      // si "adelante" da valores BAJOS en vez de altos
    int full_scale = 450;       // recorrido desde la zona muerta hasta el tope (5 V: 450, 3.3 V: ~290)
    double max_speed = 30.0;    // cm/s con el stick a fondo
};

struct CameraConfig {
    int index = 2;
    int width = 640;
    int height = 480;
    double fps = 30.0;
    std::string calibration_file = "./data/camera_calibration_params.xml";
    bool undistort = true;
};

struct VisionConfig {
    std::string color_file = "./data/color_params_data.xml";
    int min_ball_area = 150;
    double min_circularity = 0.70;
    bool flip_x = false;        // invierte el eje X de la homografía
    bool flip_y = false;
    // Centros ArUco en cm: TL, TR, BL, BR tal como se ven en la imagen
    std::vector<cv::Point2f> markers = {
        {-20.4f, 8.15f}, {20.4f, 8.15f}, {-20.4f, -8.15f}, {20.4f, -8.15f}};
    double table_half_width = 8.15;   // paredes laterales (Y) para los rebotes
    double table_half_length = 24.0;  // solo para dibujar / simular
    double velocity_alpha = 0.5;      // suavizado de la velocidad (0..1)
    double lost_timeout_s = 0.3;      // sin bola este tiempo => velocidad reseteada
    bool predict = true;              // IA: predice el punto de intercepción
};

struct GameConfig {
    int points_to_win = 5;
    bool auto_goals = true;
    int goal_frames = 3;              // frames seguidos detrás de la línea
    double goal_max_abs_y = 9.0;      // bola en la esquina (|Y| mayor) => no es gol
    double rearm_center_u = 10.0;     // la bola debe volver a |X| < esto
    std::string scores_file = "data/scores.csv";
};

// ---------------------------------------------------------------------------
// Ventiladores (Arduino, orden "F<mask>"): uno DETRÁS de cada brazo.
// Si la bola se queda trancada en la zona central, se enciende el ventilador
// del lado donde está la bola (mitad del brazo A -> el de A, mitad de B -> el de B)
// para devolverla al juego. Nunca los dos a la vez.
// ---------------------------------------------------------------------------
struct FanConfig {
    bool enabled = false;
    std::string device = "/dev/ttyACM0";   // el mismo Arduino de los joysticks
    int fan_pos_x = 0;           // bit del ventilador detrás del brazo A (+X): 0 = pin D5
    int fan_neg_x = 3;           // bit del ventilador detrás del brazo B (-X): 3 = pin D10
    double middle_x = 26.0;      // zona central |X| <= esto (más allá actúa el brazo)
    double stuck_radius = 1.5;   // cm: se ha movido menos que esto ...
    double stuck_time_s = 1.2;   // ... durante este tiempo => trancada
    double min_on_s = 0.8;       // tiempo mínimo encendido
    double max_on_s = 3.0;       // máximo seguido ...
    double rest_s = 1.5;         // ... y luego descansa
    double refresh_s = 0.3;      // reenvío periódico (el Arduino apaga a los 1.5 s)
};

struct Config {
    CameraConfig camera;
    VisionConfig vision;
    GameConfig game;
    std::vector<ArmConfig> arms;            // normalmente A (P1) y B (P2)
    std::vector<JoystickConfig> joysticks;  // [0] = P1, [1] = P2
    FanConfig fans;

    // Busca un brazo por nombre; nullptr si no existe.
    const ArmConfig* arm(const std::string& name) const;
};

// Carga el YAML. Si falta un campo se usa el valor por defecto de arriba.
bool loadConfig(const std::string& path, Config& cfg, std::string& error);

}  // namespace pongbot

#endif
