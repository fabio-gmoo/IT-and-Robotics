// =============================================================================
//  PongBot - programa principal (un solo ejecutable para todos los modos)
//
//    ./bin/pongbot                 -> GUI: check de hardware, menú, partida
//    ./bin/pongbot --mode 1        -> entra directo al modo 1
//    ./bin/pongbot --sim           -> sin hardware (bola y brazos simulados)
//    ./bin/pongbot --no-motors     -> cámara real, brazos NO se mueven
//    ./bin/pongbot --arm B         -> brazo usado en los modos de 1 brazo
//    ./bin/pongbot --config otra.yaml
//
//  Modos:
//    1. IA (1 brazo)            2. JOYSTICK (1 brazo)
//    3. JOYSTICK vs IA          4. JOYSTICK vs JOYSTICK      5. IA vs IA
//  P1 = primer brazo del YAML (A), P2 = segundo brazo (B).
// =============================================================================
#include <climits>
#include <cstdlib>
#include <unistd.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "pongbot/arm.hpp"
#include "pongbot/config.hpp"
#include "pongbot/controllers.hpp"
#include "pongbot/game_gui.hpp"
#include "pongbot/joystick.hpp"
#include "pongbot/score_manager.hpp"
#include "pongbot/sim.hpp"
#include "pongbot/vision.hpp"

using namespace pongbot;

namespace {

const std::vector<std::string> MODE_NAMES = {
    "IA (1 brazo)",
    "JOYSTICK (1 brazo)",
    "JOYSTICK vs IA",
    "JOYSTICK vs JOYSTICK",
    "IA vs IA",
};
enum Mode { MODE_AI = 1, MODE_JOY = 2, MODE_JOY_VS_AI = 3, MODE_JOY_VS_JOY = 4, MODE_AI_VS_AI = 5 };

struct Options {
    std::string config = "config/pongbot.yaml";
    int mode = 0;
    bool sim = false;
    bool no_motors = false;
    std::string single_arm;
};

void usage() {
    std::cout << "Uso: ./bin/pongbot [--config FILE] [--mode 1-5] [--sim] [--no-motors] [--arm A|B]\n"
              << "  Modos: 1 IA | 2 Joystick | 3 Joy vs IA | 4 Joy vs Joy | 5 IA vs IA\n";
}

bool parseArgs(int argc, char** argv, Options& o) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" && i + 1 < argc) o.config = argv[++i];
        else if (a == "--mode" && i + 1 < argc) o.mode = std::atoi(argv[++i]);
        else if (a == "--sim") o.sim = true;
        else if (a == "--no-motors") o.no_motors = true;
        else if (a == "--arm" && i + 1 < argc) o.single_arm = argv[++i];
        else if (a == "-h" || a == "--help") { usage(); return false; }
        else { std::cerr << "Argumento desconocido: " << a << "\n"; usage(); return false; }
    }
    if (o.mode < 0 || o.mode > 5) { usage(); return false; }
    return true;
}

// "/dev/serial/by-id/usb-FTDI_..._FT4TFLHO-if00-port0" -> "ttyUSB1 - FT4TFLHO"
std::string shortDevice(const std::string& path) {
    const std::string byid = "/dev/serial/by-id/";
    if (path.rfind(byid, 0) != 0) return path;
    std::string name = path.substr(byid.size());
    for (const char* suf : {"-if00-port0", "-if00"}) {
        const size_t p = name.rfind(suf);
        if (p != std::string::npos && p + std::strlen(suf) == name.size()) name.erase(p);
    }
    const size_t us = name.rfind('_');
    std::string serial = us == std::string::npos ? name : name.substr(us + 1);
    if (serial.size() > 10) serial = "..." + serial.substr(serial.size() - 6);
    char real[PATH_MAX];
    std::string tty = realpath(path.c_str(), real) ? std::string(real) : "";
    const size_t sl = tty.rfind('/');
    if (sl != std::string::npos) tty = tty.substr(sl + 1);
    return (tty.empty() ? "desconectado" : tty) + " - " + serial;
}

double secondsSince(Clock::time_point t0) {
    return std::chrono::duration<double>(Clock::now() - t0).count();
}

// Detecta goles: la bola pasa la línea de gol de un lado varios frames seguidos.
class GoalDetector {
public:
    // Devuelve el lado (+1/-1) donde entró el gol, o 0.
    int update(const BallState& b, double goal_u_pos, double goal_u_neg,
               int frames_needed, double rearm_u, double t) {
        if (b.valid) last_seen_ = t;
        if (locked_) {
            const bool back_in_center = b.valid && std::fabs(b.pos.x) < rearm_u;
            const bool gone = (t - last_seen_) > 1.5;
            if (back_in_center || gone) locked_ = false;
            return 0;
        }
        if (!b.valid) return 0;
        for (int side : {1, -1}) {
            const double gu = side > 0 ? goal_u_pos : goal_u_neg;
            int& cnt = side > 0 ? cnt_pos_ : cnt_neg_;
            if (side * b.pos.x > gu) ++cnt;
            else cnt = 0;
            if (cnt >= frames_needed) {
                cnt_pos_ = cnt_neg_ = 0;
                locked_ = true;
                return side;
            }
        }
        return 0;
    }
    void reset() { cnt_pos_ = cnt_neg_ = 0; locked_ = false; }

private:
    int cnt_pos_ = 0, cnt_neg_ = 0;
    bool locked_ = false;
    double last_seen_ = 0.0;
};

struct Player {
    PongArm* arm = nullptr;
    std::unique_ptr<Controller> ctrl;
    std::string name;
};

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) return 1;

    Config cfg;
    std::string err;
    if (!loadConfig(opt.config, cfg, err)) {
        std::cerr << "[CONFIG] " << err << "\n";
        return 1;
    }
    std::cout << "[CONFIG] " << opt.config << " cargado: " << cfg.arms.size() << " brazo(s), "
              << cfg.joysticks.size() << " joystick(s)\n";
    const bool motors = !opt.sim && !opt.no_motors;

    // ---------------- Hardware ----------------
    std::vector<std::unique_ptr<PongArm>> arms;
    HardwareStatus hw;
    CameraThread camera;
    TableVision vision(cfg.vision);
    SimTable sim(cfg.vision);

    if (!vision.loadColor(err)) {
        if (opt.sim) std::cerr << "[VISION] " << err << " (ignorado en --sim)\n";
        else { std::cerr << "[VISION] " << err << "\n"; return 1; }
    }
    vision.loadCameraCalibration(cfg.camera.calibration_file, cfg.camera.undistort);

    auto scanHardware = [&]() {
        hw = HardwareStatus{};
        // Cámara
        if (opt.sim) {
            hw.devices.push_back({"Camara", "simulada", true, ""});
            hw.cameraOk = true;
        } else {
            if (!camera.running()) {
                std::string info;
                camera.start(cfg.camera, info);
                std::cout << "[CAMERA] " << info << "\n";
                hw.devices.push_back({"Camara", "video " + std::to_string(camera.running()
                                                                              ? camera.openedIndex() : cfg.camera.index),
                                      camera.running(), info});
            } else {
                hw.devices.push_back({"Camara", "video " + std::to_string(camera.openedIndex()), true, ""});
            }
            hw.cameraOk = camera.running();
        }
        // Brazos (se reconectan los que fallaron)
        if (arms.empty())
            for (const auto& ac : cfg.arms)
                if (ac.enabled) arms.push_back(std::make_unique<PongArm>(ac, !motors));
        bool any_arm = false;
        for (auto& a : arms) {
            std::string e;
            bool ok = a->isSimulated() || a->isConnected() || a->connect(e);
            if (!ok) {
                // Si falló, recrear para poder reintentar con R
                a = std::make_unique<PongArm>(a->config(), false);
                std::cerr << "[ARM " << a->config().name << "] " << e << "\n";
            }
            any_arm |= ok;
            hw.devices.push_back({"Brazo " + a->config().name,
                                  a->isSimulated() ? "simulado" : shortDevice(a->config().port), ok, e});
        }
        hw.dynamixelOk = any_arm;
        // Joysticks (solo comprobamos que el dispositivo existe)
        for (size_t i = 0; i < cfg.joysticks.size(); ++i) {
            const auto& j = cfg.joysticks[i];
            const bool ok = access(j.device.c_str(), R_OK | W_OK) == 0;
            hw.devices.push_back({"Joystick P" + std::to_string(i + 1), shortDevice(j.device) + " - stick " +
                                  std::to_string(j.channel + 1), ok, ok ? "" : "sin acceso -> se usa el teclado"});
            hw.joystickOk |= ok;
        }
    };
    scanHardware();

    PongArm* armA = arms.size() > 0 ? arms[0].get() : nullptr;
    PongArm* armB = arms.size() > 1 ? arms[1].get() : nullptr;
    auto findArm = [&](const std::string& name) -> PongArm* {
        for (auto& a : arms) if (a->config().name == name) return a.get();
        return nullptr;
    };

    // ---------------- GUI ----------------
    const int WIDTH = 960, HEIGHT = 540;
    cv::Mat canvas(HEIGHT, WIDTH, CV_8UC3);
    GameGui gui(WIDTH, HEIGHT);
    ScoreManager scores(cfg.game.scores_file);
    const std::string WIN = "PongBot";
    cv::namedWindow(WIN, cv::WINDOW_AUTOSIZE);

    GuiState state = GuiState::SYSTEM_CHECK;
    int selectedMode = opt.mode > 0 ? opt.mode : 1;
    std::string menuMessage;
    int scoreP1 = 0, scoreP2 = 0;
    std::string winner;
    Clock::time_point matchStart;
    Player p1, p2;
    bool showMask = false;
    GoalDetector goals;

    const auto t0 = Clock::now();
    auto lastLoop = Clock::now();
    auto lastRender = Clock::now() - std::chrono::seconds(1);
    auto lastSimInject = Clock::now();
    long lastFrameId = 0;
    cv::Mat frame, debugFrame;

    // --- Configura una partida según el modo ---
    auto startMatch = [&](int mode) -> bool {
        for (auto& a : arms) { a->setArmed(false); a->goHome(); }
        p1 = Player{};
        p2 = Player{};
        const bool twoArms = mode >= MODE_JOY_VS_AI;
        PongArm* single = opt.single_arm.empty() ? armA : findArm(opt.single_arm);
        if (!single) { menuMessage = "No hay brazo " + opt.single_arm + " en el YAML"; return false; }
        if (twoArms && !armB) { menuMessage = "Este modo necesita 2 brazos habilitados en el YAML"; return false; }
        if (!twoArms && !(single->isConnected() || single->isSimulated())) {
            menuMessage = "Brazo " + single->config().name + " no conectado (pulsa R)"; return false;
        }
        if (twoArms && (!(armA->isConnected() || armA->isSimulated()) ||
                        !(armB->isConnected() || armB->isSimulated()))) {
            menuMessage = "Algun brazo no esta conectado (pulsa R en el menu)"; return false;
        }

        const JoystickConfig j1 = cfg.joysticks.size() > 0 ? cfg.joysticks[0] : JoystickConfig{};
        JoystickConfig j2 = cfg.joysticks.size() > 1 ? cfg.joysticks[1] : JoystickConfig{};
        if (cfg.joysticks.size() < 2) j2.device = "none";
        const KeyBinding k1{'a', 'd', 'w'};
        const KeyBinding k2{'j', 'l', 'i'};
        const bool pred = cfg.vision.predict;

        auto mk = [&](bool ai, const JoystickConfig& j, KeyBinding k) -> std::unique_ptr<Controller> {
            if (ai) return std::make_unique<AIController>(pred);
            auto jc = std::make_unique<JoystickController>(j, k);
            if (!jc->hasJoystick())
                std::cerr << "[JOY] " << jc->openError() << " -> usando teclado\n";
            return jc;
        };

        switch (mode) {
            case MODE_AI:
                p1 = {single, mk(true, j1, k1), "IA " + single->config().name};
                break;
            case MODE_JOY:
                p1 = {single, mk(false, j1, k1), "P1"};
                break;
            case MODE_JOY_VS_AI:
                p1 = {armA, mk(false, j1, k1), "P1"};
                p2 = {armB, mk(true, j2, k2), "IA"};
                break;
            case MODE_JOY_VS_JOY:
                p1 = {armA, mk(false, j1, k1), "P1"};
                p2 = {armB, mk(false, j2, k2), "P2"};
                break;
            case MODE_AI_VS_AI:
                p1 = {armA, mk(true, j1, k1), "IA A"};
                p2 = {armB, mk(true, j2, k2), "IA B"};
                break;
        }
        if (!p2.arm) p2.name = "PARED";
        if (p1.arm) p1.arm->setArmed(true);
        if (p2.arm) p2.arm->setArmed(true);
        scoreP1 = scoreP2 = 0;
        goals.reset();
        if (opt.sim) sim.serve();
        matchStart = Clock::now();
        menuMessage.clear();
        std::cout << "[MATCH] Modo " << mode << ": " << MODE_NAMES[mode - 1] << "\n";
        return true;
    };

    if (opt.mode > 0) {
        if (startMatch(opt.mode)) state = GuiState::PLAYING_MATCH;
        else { std::cerr << "[MATCH] " << menuMessage << "\n"; state = GuiState::MAIN_MENU; }
    }

    // ======================= BUCLE PRINCIPAL =======================
    while (true) {
        const auto now = Clock::now();
        const double dt = std::min(0.1, std::chrono::duration<double>(now - lastLoop).count());
        lastLoop = now;
        const double t = secondsSince(t0);
        bool newFrame = false;

        // ---- 1) Percepción ----
        if (opt.sim) {
            if (state == GuiState::PLAYING_MATCH) {
                std::vector<const PongArm*> playing;
                if (p1.arm) playing.push_back(p1.arm);
                if (p2.arm) playing.push_back(p2.arm);
                sim.step(dt, playing);
            }
            if (std::chrono::duration<double>(now - lastSimInject).count() >= 1.0 / 30.0) {
                vision.injectBall(sim.visible(), sim.pos(), t);
                lastSimInject = now;
                newFrame = true;
            }
        } else if (camera.latest(frame, lastFrameId)) {
            vision.process(frame, t, debugFrame);
            newFrame = true;
        }

        // ---- 2) Control ----
        if (state == GuiState::PLAYING_MATCH) {
            if (p1.arm && p1.ctrl) p1.ctrl->update(*p1.arm, vision, dt);
            if (p2.arm && p2.ctrl) p2.ctrl->update(*p2.arm, vision, dt);

            // Goles automáticos
            if (cfg.game.auto_goals && newFrame) {
                double gpos = 23.5, gneg = 23.5;
                for (auto& a : arms) {
                    if (a->config().side > 0) gpos = a->config().goal_u;
                    else gneg = a->config().goal_u;
                }
                const int side = goals.update(vision.ball(), gpos, gneg, cfg.game.goal_frames,
                                              cfg.game.rearm_center_u, t);
                if (side != 0) {
                    // Gol en el lado de P1 => punto para P2, y viceversa
                    const int p1side = p1.arm ? p1.arm->config().side : 1;
                    if (side == p1side) ++scoreP2; else ++scoreP1;
                    std::cout << "[GOAL] " << (side == p1side ? p2.name : p1.name) << "  " << scoreP1
                              << " - " << scoreP2 << "\n";
                }
            }
        }
        for (auto& a : arms) a->update();   // no bloquea

        // ---- 3) Fin de partida ----
        if (state == GuiState::PLAYING_MATCH &&
            (scoreP1 >= cfg.game.points_to_win || scoreP2 >= cfg.game.points_to_win)) {
            const int totalSec = static_cast<int>(secondsSince(matchStart));
            winner = scoreP1 > scoreP2 ? p1.name : p2.name;
            scores.saveMatch(MODE_NAMES[selectedMode - 1], scoreP1, scoreP2, winner, totalSec);
            for (auto& a : arms) { a->setArmed(false); a->goHome(); }
            state = GuiState::GAME_OVER;
        }

        // ---- 4) Dibujo (~30 Hz) ----
        if (std::chrono::duration<double>(now - lastRender).count() >= 1.0 / 30.0) {
            lastRender = now;
            if (state == GuiState::SYSTEM_CHECK) {
                gui.drawSystemCheckScreen(canvas, hw, opt.sim);
            } else if (state == GuiState::MAIN_MENU) {
                gui.drawMainMenuScreen(canvas, selectedMode, MODE_NAMES, hw,
                                       scores.loadRecentMatches(5), menuMessage);
            } else if (state == GuiState::PLAYING_MATCH) {
                canvas = cv::Scalar(25, 25, 30);
                // Izquierda: cámara (o aviso en simulación)
                const cv::Rect camArea(0, 72, 600, 442);
                if (!opt.sim && !debugFrame.empty()) {
                    cv::Mat small;
                    const double s = std::min(camArea.width / double(debugFrame.cols),
                                              camArea.height / double(debugFrame.rows));
                    cv::resize(debugFrame, small, cv::Size(), s, s);
                    small.copyTo(canvas(cv::Rect(camArea.x, camArea.y, small.cols, small.rows)));
                } else {
                    gui.drawRetroText(canvas, opt.sim ? "[SIMULACION]" : "[Camara sin imagen]",
                                      cv::Point(200, 290), 0.7, cv::Scalar(120, 120, 120), 1);
                }
                if (!opt.sim && !vision.homographyReady())
                    gui.drawRetroText(canvas, "Mostrar los 4 ArUco a la camara", cv::Point(20, 110),
                                      0.7, cv::Scalar(0, 200, 255), 2);
                // Derecha: vista cenital + estado de cada brazo
                PlanView pv;
                pv.half_length = cfg.vision.table_half_length;
                pv.half_width = cfg.vision.table_half_width;
                pv.markers = cfg.vision.markers;
                pv.ball_valid = vision.ball().valid;
                pv.ball = vision.ball().pos;
                pv.vel = vision.ball().vel;
                pv.vel_valid = vision.ball().vel_valid;
                for (Player* p : {&p1, &p2}) {
                    if (!p->arm) continue;
                    PlanArm pa;
                    pa.label = p->name;
                    pa.side = p->arm->config().side;
                    pa.paddle_u = p->arm->paddleU();
                    pa.paddle_y = p->arm->paddleTableY();
                    pa.goal_u = p->arm->config().goal_u;
                    pa.smashing = p->arm->isSmashing();
                    pa.armed = p->arm->armed();
                    pv.arms.push_back(pa);
                }
                gui.drawPlan(canvas, cv::Rect(608, 80, 344, 180), pv);
                int y = 290;
                for (Player* p : {&p1, &p2}) {
                    if (!p->arm) continue;
                    char buf[160];
                    std::snprintf(buf, sizeof(buf), "%s [%s] brazo %s %s", p->name.c_str(),
                                  p->ctrl ? p->ctrl->label().c_str() : "-", p->arm->config().name.c_str(),
                                  p->arm->armed() ? "ARMADO" : "DESARMADO");
                    cv::putText(canvas, buf, cv::Point(612, y), cv::FONT_HERSHEY_SIMPLEX, 0.45,
                                cv::Scalar(230, 230, 230), 1, cv::LINE_AA);
                    std::snprintf(buf, sizeof(buf), "  x=%.1f y=%.1f %s err=%d", p->arm->currentX(),
                                  p->arm->currentY(), smashStateName(p->arm->smashState()),
                                  p->arm->sendErrors());
                    cv::putText(canvas, buf, cv::Point(612, y + 18), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                                cv::Scalar(180, 220, 180), 1, cv::LINE_AA);
                    if (p->ctrl)
                        cv::putText(canvas, "  " + p->ctrl->status(), cv::Point(612, y + 36),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.42, cv::Scalar(180, 180, 220), 1, cv::LINE_AA);
                    y += 64;
                }
                cv::putText(canvas, "Teclado: P1 A/D + W smash   P2 J/L + I smash", cv::Point(612, 495),
                            cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(160, 160, 160), 1, cv::LINE_AA);
                gui.drawMatchHud(canvas, scoreP1, scoreP2, static_cast<int>(secondsSince(matchStart)),
                                 MODE_NAMES[selectedMode - 1] + (opt.sim ? "  (SIM)" : "") +
                                     (opt.no_motors ? "  (SIN MOTORES)" : ""),
                                 p1.name, p2.name);
            } else if (state == GuiState::GAME_OVER) {
                gui.drawGameOverScreen(canvas, winner, scoreP1, scoreP2);
            }
            cv::imshow(WIN, canvas);
            if (showMask && !vision.mask().empty()) cv::imshow("Ball Mask", vision.mask());
        }

        // ---- 5) Teclado ----
        const int key = cv::waitKey(1);
        if (key < 0) continue;
        if (key == 27) break;  // ESC

        if (state == GuiState::SYSTEM_CHECK) {
            if (key == 'r' || key == 'R') scanHardware();
            else if (key == 13 || key == 10 || key == 32) state = GuiState::MAIN_MENU;
        } else if (state == GuiState::MAIN_MENU) {
            const int n = static_cast<int>(MODE_NAMES.size());
            if (key == 82 || key == 'w') selectedMode = selectedMode > 1 ? selectedMode - 1 : n;
            else if (key == 84 || key == 's') selectedMode = selectedMode < n ? selectedMode + 1 : 1;
            else if (key >= '1' && key < '1' + n) selectedMode = key - '0';
            else if (key == 'r') { scanHardware(); state = GuiState::SYSTEM_CHECK; }
            else if (key == 13 || key == 10 || key == 32) {
                armA = arms.size() > 0 ? arms[0].get() : nullptr;   // pueden haberse recreado
                armB = arms.size() > 1 ? arms[1].get() : nullptr;
                if (startMatch(selectedMode)) state = GuiState::PLAYING_MATCH;
            }
        } else if (state == GuiState::PLAYING_MATCH) {
            if (key == 'm') {
                for (auto& a : arms) { a->setArmed(false); a->goHome(); }
                state = GuiState::MAIN_MENU;
            } else if (key == 32) ++scoreP1;
            else if (key == 'b') ++scoreP2;
            else if (key == 'x') {
                for (Player* p : {&p1, &p2})
                    if (p->arm) p->arm->setArmed(!p->arm->armed());
            } else if (key == 'h') {
                for (auto& a : arms) a->goHome();
            } else if (key == 'c') {
                vision.resetHomography();
                std::cout << "[VISION] Homografía borrada: mostrar los 4 ArUco.\n";
            } else if (key == 'v') {
                showMask = !showMask;
                if (!showMask) cv::destroyWindow("Ball Mask");
            } else {
                if (p1.ctrl) p1.ctrl->handleKey(key);
                if (p2.ctrl) p2.ctrl->handleKey(key);
            }
        } else if (state == GuiState::GAME_OVER) {
            if (key == 13 || key == 10 || key == 32) state = GuiState::MAIN_MENU;
        }
    }

    // ---- Salida limpia: brazos a home (el torque se queda activo, como antes) ----
    for (auto& a : arms) { a->setArmed(false); a->goHome(); }
    camera.stop();
    cv::destroyAllWindows();
    std::cout << "[PONGBOT] Fin.\n";
    return 0;
}
