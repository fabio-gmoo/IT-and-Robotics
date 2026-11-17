// =============================================================================
//  arm_test: calibrar y probar UN brazo, sin cámara.
//  Reúne interactive_test.cpp y main_robot_test.cpp, pero usando el YAML,
//  así sirve igual para el brazo A y el B.
//
//    ./bin/arm_test --arm A          (o --arm B)
//    ./bin/arm_test --arm B --config config/pongbot.yaml
// =============================================================================
#include <unistd.h>

#include <cmath>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

#include "pongbot/arm.hpp"
#include "pongbot/config.hpp"

using namespace pongbot;

// Lee números sin que una letra cierre el programa
template <typename... T>
static bool readNums(T&... v) {
    if ((std::cin >> ... >> v)) return true;
    std::cin.clear();
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
    std::cout << "  Entrada inválida (se esperaban números). Volviendo al menú.\n";
    return false;
}

static void sleepMs(int ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

// Ejecuta update() durante 'ms' para que el smash avance (no bloqueante por dentro)
static void runFor(PongArm& arm, int ms) {
    const auto t0 = Clock::now();
    while (std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - t0).count() < ms) {
        arm.update();
        sleepMs(2);
    }
}

int main(int argc, char** argv) {
    std::string cfgPath = "config/pongbot.yaml", armName = "A";
    bool sim = false;
    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--arm" && i + 1 < argc) armName = argv[++i];
        else if (a == "--config" && i + 1 < argc) cfgPath = argv[++i];
        else if (a == "--sim") sim = true;
        else { std::cout << "Uso: ./bin/arm_test [--arm A|B] [--config FILE] [--sim]\n"; return 1; }
    }

    Config cfg;
    std::string err;
    if (!loadConfig(cfgPath, cfg, err)) { std::cerr << err << "\n"; return 1; }
    const ArmConfig* ac = cfg.arm(armName);
    if (!ac) { std::cerr << "No existe el brazo '" << armName << "' en " << cfgPath << "\n"; return 1; }

    PongArm arm(*ac, sim);
    if (!arm.connect(err)) { std::cerr << "[-] " << err << "\n"; return 1; }
    const ArmConfig& c = arm.config();

    std::cout << "\nBrazo " << c.name << " en " << c.port << "  IDs base=" << c.id_base
              << " hombro=" << c.id_shoulder << " codo=" << c.id_elbow << " muñeca=" << c.id_wrist
              << "\n  dir=[" << c.dir[0] << "," << c.dir[1] << "," << c.dir[2] << "]  side=" << c.side
              << "  lateral_sign=" << c.lateral_sign << "  home_y=" << c.home_y << "\n";

    while (true) {
        std::cout << "\n================ ARM TEST (" << c.name << ") ================\n"
                  << " [1] Home (x=0, y=home_y)\n"
                  << " [2] Barrido lateral en home_y (izq -> der)  <- comprobar sentido\n"
                  << " [3] Secuencia original main_robot_test (centro/izq/centro/der)\n"
                  << " [4] Smash en la x actual (no bloqueante)\n"
                  << " [5] Ir a (x, y) con IK\n"
                  << " [6] Ángulos articulares en grados (base, codo, muñeca)\n"
                  << " [7] Ticks crudos a un motor (ID, ticks)\n"
                  << " [8] Alcance máximo de Y para cada x (tabla)\n"
                  << " [9] Ajustar HOMBRO fijo (altura / eje Z del plano)\n"
                  << " [q] Salir\nOpción: ";
        std::string op;
        if (!(std::cin >> op) || op == "q" || op == "Q") break;

        if (op == "1") {
            arm.goHome();
        } else if (op == "2") {
            std::cout << "La pala debería ir de IZQUIERDA a DERECHA vista DESDE el robot.\n"
                         "Si va al revés -> cambia dir[0] (base) o lateral_sign en el YAML.\n";
            std::cout << "Fíjate en qué x toca la pared -> pon lateral_min (izq) / lateral_max (der)\n"
                         "un poco más hacia dentro (p.ej. si toca en x=-6, lateral_min: -5.5).\n";
            for (double x = c.lateral_min; x <= c.lateral_max + 1e-6; x += 0.5) {
                std::cout << "  x=" << x << std::flush;
                if (!arm.reachable(x, c.home_y)) std::cout << "  fuera de alcance";
                else arm.moveToPlanar(x, c.home_y);
                std::cout << "\n";
                sleepMs(250);
            }
            arm.goHome();
        } else if (op == "3") {
            const double pts[5][2] = {{0.0, 16.5}, {-4.0, 13.0}, {0.0, 16.5}, {4.0, 13.0}, {0.0, 16.5}};
            for (auto& p : pts) {
                const bool ok = arm.reachable(p[0], p[1]);
                std::cout << "  -> (" << p[0] << ", " << p[1] << ")" << (ok ? "" : "  FUERA DE ALCANCE") << "\n";
                if (ok) arm.moveToPlanar(p[0], p[1]);
                sleepMs(2000);
            }
            arm.goHome();
        } else if (op == "4") {
            double x;
            std::cout << "x lateral para el smash (cm): ";
            if (!readNums(x)) continue;
            arm.setLateralTarget(x);
            runFor(arm, 400);
            arm.triggerSmash();
            runFor(arm, c.recoil_ms + c.strike_ms + c.return_ms + c.cooldown_ms + 100);
        } else if (op == "5") {
            double x, y;
            std::cout << "x y (cm, separados por espacio): ";
            if (!readNums(x, y)) continue;
            double q1, q2, q3;
            if (!arm.computeIK(x, y, c.gamma, q1, q2, q3)) { std::cout << "Fuera de alcance\n"; continue; }
            std::cout << "  q1=" << q1 * 180 / M_PI << "° q2=" << q2 * 180 / M_PI << "° q3=" << q3 * 180 / M_PI
                      << "°  ticks=(" << arm.jointToTicks(0, q1) << "," << arm.jointToTicks(1, q2) << ","
                      << arm.jointToTicks(2, q3) << ")\n";
            arm.moveToPlanar(x, y);
        } else if (op == "6") {
            double d[3];
            std::cout << "base codo muñeca (3 números en grados, p.ej. 30 0 0): ";
            if (!readNums(d[0], d[1], d[2])) continue;
            const int ids[3] = {c.id_base, c.id_elbow, c.id_wrist};
            for (int j = 0; j < 3; ++j) {
                const int t = arm.jointToTicks(j, d[j] * M_PI / 180.0);
                std::cout << "  ID " << ids[j] << " -> " << t << " ticks\n";
                arm.writeRaw(ids[j], t);
            }
        } else if (op == "7") {
            int id, t;
            std::cout << "ID y ticks (2 números, p.ej. 1 512): ";
            if (!readNums(id, t)) continue;
            std::cout << (arm.writeRaw(id, t) ? "  OK\n" : "  FALLO (ver mensaje de error)\n");
        } else if (op == "9") {
            int t = c.shoulder_ticks;
            std::cout << "Ajuste del hombro (ID " << c.id_shoulder << "). Actual: " << t << " ticks\n"
                      << "  +  / -    -> mover 5 ticks       ++ / --  -> mover 20 ticks\n"
                      << "  número    -> ir a ese valor      ok       -> terminar\n"
                      << "Objetivo: que la pala quede HORIZONTAL (paralela a la mesa), igual que el brazo A.\n";
            arm.writeRaw(c.id_shoulder, t);
            std::string in;
            while (std::cout << "  hombro=" << t << " > " && std::cin >> in && in != "ok") {
                if (in == "+") t += 5;
                else if (in == "-") t -= 5;
                else if (in == "++") t += 20;
                else if (in == "--") t -= 20;
                else {
                    try { t = std::stoi(in); } catch (...) { std::cout << "  ?\n"; continue; }
                }
                t = std::max(0, std::min(1023, t));
                arm.writeRaw(c.id_shoulder, t);
            }
            arm.setShoulderTicks(t);
            arm.goHome();
            std::cout << "\n>>> Pon en config/pongbot.yaml, brazo " << c.name << ":   shoulder_ticks: " << t
                      << "\n    (vuelve a probar [1] home y [2] barrido con este valor)\n";
        } else if (op == "8") {
            for (double x = 0; x <= 8.0 + 1e-6; x += 1.0)
                std::cout << "  x=±" << x << "  y_max=" << arm.maxReachableY(x, 20.0, 0.0)
                          << (arm.reachable(x, c.home_y) ? "" : "   (home_y NO alcanzable)") << "\n";
        }
    }
    arm.goHome();
    return 0;
}
