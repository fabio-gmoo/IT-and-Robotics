// =============================================================================
//  PongArm: un brazo Poppy Ergo Jr planar (3 GDL activos + hombro fijo)
//
//  - La cinemática inversa es EXACTAMENTE la de pong_robot.hpp (Valeria), que
//    es la que funcionó en la mesa. No tocar computeIK sin probar en la mesa.
//  - Nada aquí bloquea: el smash es una máquina de estados que avanza en
//    update(), así el bucle de cámara nunca se congela (adiós usleep).
//  - Varios brazos pueden compartir el mismo puerto USB (U2D2) si tienen IDs
//    distintos: DxlBus se comparte automáticamente.
// =============================================================================
#ifndef PONGBOT_ARM_HPP
#define PONGBOT_ARM_HPP

#include <array>
#include <chrono>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "pongbot/config.hpp"

namespace dynamixel {
class PortHandler;
class PacketHandler;
}  // namespace dynamixel

namespace pongbot {

using Clock = std::chrono::steady_clock;

// Puerto serie Dynamixel compartido (uno por /dev/ttyUSBx).
class DxlBus {
public:
    static std::shared_ptr<DxlBus> open(const std::string& port, int baudrate,
                                        std::string& error);
    ~DxlBus();

    bool write1(int id, int addr, int value);
    bool write2(int id, int addr, int value);
    bool read2(int id, int addr, int& value);
    const std::string& port() const { return port_; }

private:
    DxlBus() = default;
    std::string port_;
    dynamixel::PortHandler* portHandler_ = nullptr;
    dynamixel::PacketHandler* packetHandler_ = nullptr;
    std::mutex mutex_;
    static std::map<std::string, std::weak_ptr<DxlBus>>& registry();
};

class PongArm {
public:
    enum class SmashState { IDLE, RECOIL, STRIKE, RETURN, COOLDOWN };

    // simulate = true -> no habla con los motores (modo --sim / --no-motors)
    PongArm(const ArmConfig& cfg, bool simulate);

    // Abre el puerto, activa torque y va a home. false si falla el hardware.
    bool connect(std::string& error);
    bool isConnected() const { return bus_ != nullptr; }
    bool isSimulated() const { return simulate_; }
    const ArmConfig& config() const { return cfg_; }

    // ---- Cinemática (idéntica a pong_robot.hpp) ----
    bool computeIK(double x, double y, double gamma_rad,
                   double& q1, double& q2, double& q3) const;
    bool reachable(double x, double y) const;
    // Mayor y <= desired_y alcanzable en x (nunca menos que min_y).
    double maxReachableY(double x, double desired_y, double min_y) const;

    // Conversión rad -> ticks con dir/offset/límites del YAML.
    int jointToTicks(int joint, double rad) const;  // joint: 0 base, 1 codo, 2 muñeca

    // ---- Órdenes de alto nivel (las dan los controladores) ----
    void setLateralTarget(double x);        // coordenada x del robot (cm)
    bool triggerSmash();                    // false si ya está golpeando / cooldown
    void setArmed(bool armed);              // desarmado = se queda en home
    bool armed() const { return armed_; }

    // Llamar en cada vuelta del bucle principal. Avanza el smash y envía a
    // los motores solo si algo cambió (y respetando min_send_ms).
    void update();
    // Lleva el brazo a home (x=0, home_y) inmediatamente.
    void goHome();

    // ---- Acceso directo (herramientas de calibración) ----
    bool moveToPlanar(double x, double y);  // como el moveToPlanar original
    bool writeRaw(int id, int ticks);

    // ---- Estado (GUI / simulación) ----
    double currentX() const { return cur_x_; }
    double currentY() const { return cur_y_; }
    SmashState smashState() const { return smash_; }
    bool isSmashing() const {
        return smash_ == SmashState::RECOIL || smash_ == SmashState::STRIKE;
    }
    int sendErrors() const { return send_errors_; }
    static bool& verbose() { static bool v = true; return v; }  // log de cada smash
    // Solo para tests/simulación acelerada: sustituye el reloj real.
    void setClock(std::function<Clock::time_point()> clock) { clock_ = std::move(clock); }
    // u de mesa (= side * X) de la pala en este momento
    double paddleU() const { return cfg_.paddle_u - (cur_y_ - cfg_.home_y); }
    // Y de mesa de la pala en este momento
    double paddleTableY() const {
        return cfg_.lateral_center + cur_x_ / cfg_.lateral_sign;
    }

private:
    bool send(double x, double y, bool force);

    ArmConfig cfg_;
    bool simulate_;
    std::shared_ptr<DxlBus> bus_;
    bool armed_ = true;

    double target_x_ = 0.0;
    double cur_x_ = 0.0, cur_y_ = 0.0;
    double smash_x_ = 0.0, smash_strike_y_ = 0.0;
    SmashState smash_ = SmashState::IDLE;
    Clock::time_point smash_t0_;

    std::array<int, 3> last_ticks_ = {-1, -1, -1};
    Clock::time_point last_send_;
    Clock::time_point last_shoulder_;
    int send_errors_ = 0;
    std::function<Clock::time_point()> clock_ = [] { return Clock::now(); };
    Clock::time_point last_error_print_;
};

const char* smashStateName(PongArm::SmashState s);

}  // namespace pongbot

#endif
