#include "pongbot/arm.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <thread>

#include "dynamixel_sdk/dynamixel_sdk.h"

namespace pongbot {

// XL-320, protocolo 2.0 (mismas direcciones que pong_robot.hpp)
constexpr double PROTOCOL_VERSION = 2.0;
constexpr int ADDR_TORQUE_ENABLE = 24;
constexpr int ADDR_GOAL_POSITION = 30;
constexpr int ADDR_PRESENT_POSITION = 37;

// ============================================================================
// DxlBus
// ============================================================================
std::map<std::string, std::weak_ptr<DxlBus>>& DxlBus::registry() {
    static std::map<std::string, std::weak_ptr<DxlBus>> r;
    return r;
}

std::shared_ptr<DxlBus> DxlBus::open(const std::string& port, int baudrate,
                                     std::string& error) {
    auto& reg = registry();
    auto it = reg.find(port);
    if (it != reg.end())
        if (auto existing = it->second.lock()) return existing;

    std::shared_ptr<DxlBus> bus(new DxlBus());
    bus->port_ = port;
    bus->portHandler_ = dynamixel::PortHandler::getPortHandler(port.c_str());
    bus->packetHandler_ = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    if (!bus->portHandler_->openPort()) {
        error = "Cannot open " + port + " (plugged in? user in 'dialout' group? USB passed to the VM?)";
        return nullptr;
    }
    if (!bus->portHandler_->setBaudRate(baudrate)) {
        error = "Cannot set baudrate " + std::to_string(baudrate) + " on " + port;
        return nullptr;
    }
    reg[port] = bus;
    return bus;
}

DxlBus::~DxlBus() {
    if (portHandler_) {
        portHandler_->closePort();
        delete portHandler_;
    }
}

// Comprueba el resultado. El bit 7 del byte de error (0x80, "hardware alert",
// p.ej. sobrecarga) NO impide que la escritura se aplique: no lo contamos como fallo.
bool DxlBus::check(int id, int comm, uint8_t err) {
    if (comm != COMM_SUCCESS) {
        last_error_ = "ID " + std::to_string(id) + ": " + packetHandler_->getTxRxResult(comm);
        return false;
    }
    if (err & 0x7F) {
        last_error_ = "ID " + std::to_string(id) + ": " + packetHandler_->getRxPacketError(err);
        return false;
    }
    if (err & 0x80) last_error_ = "ID " + std::to_string(id) + ": hardware alert (sobrecarga/temperatura?)";
    return true;
}

bool DxlBus::write1(int id, int addr, int value) {
    std::lock_guard<std::mutex> lock(mutex_);
    uint8_t err = 0;
    int res = packetHandler_->write1ByteTxRx(portHandler_, id, addr,
                                             static_cast<uint8_t>(value), &err);
    if (write_gap_us > 0) std::this_thread::sleep_for(std::chrono::microseconds(write_gap_us));
    return check(id, res, err);
}

bool DxlBus::write2(int id, int addr, int value) {
    std::lock_guard<std::mutex> lock(mutex_);
    uint8_t err = 0;
    int res = packetHandler_->write2ByteTxRx(portHandler_, id, addr,
                                             static_cast<uint16_t>(value), &err);
    if (write_gap_us > 0) std::this_thread::sleep_for(std::chrono::microseconds(write_gap_us));
    return check(id, res, err);
}

bool DxlBus::read2(int id, int addr, int& value) {
    std::lock_guard<std::mutex> lock(mutex_);
    uint8_t err = 0;
    uint16_t v = 0;
    int res = packetHandler_->read2ByteTxRx(portHandler_, id, addr, &v, &err);
    value = v;
    return res == COMM_SUCCESS && err == 0;
}

// ============================================================================
// PongArm
// ============================================================================
const char* smashStateName(PongArm::SmashState s) {
    switch (s) {
        case PongArm::SmashState::IDLE: return "IDLE";
        case PongArm::SmashState::RECOIL: return "RECOIL";
        case PongArm::SmashState::STRIKE: return "STRIKE";
        case PongArm::SmashState::RETURN: return "RETURN";
        case PongArm::SmashState::COOLDOWN: return "COOLDOWN";
    }
    return "?";
}

PongArm::PongArm(const ArmConfig& cfg, bool simulate)
    : cfg_(cfg), simulate_(simulate) {
    cur_x_ = 0.0;
    cur_y_ = cfg_.home_y;
    last_send_ = last_shoulder_ = last_error_print_ = Clock::now() - std::chrono::seconds(10);
}

bool PongArm::connect(std::string& error) {
    if (simulate_) return true;
    bus_ = DxlBus::open(cfg_.port, cfg_.baudrate, error);
    if (!bus_) return false;
    bus_->write_gap_us = cfg_.write_gap_us;

    const int ids[4] = {cfg_.id_base, cfg_.id_shoulder, cfg_.id_elbow, cfg_.id_wrist};
    for (int id : ids) {
        if (!bus_->write1(id, ADDR_TORQUE_ENABLE, 1)) {
            error = "Motor ID " + std::to_string(id) + " of arm " + cfg_.name +
                    " not responding [" + bus_->lastError() + "] (right ID? power? baudrate?)";
            bus_.reset();
            return false;
        }
    }
    goHome();
    std::cout << "[ARM " << cfg_.name << "] Conectado en " << cfg_.port << " (IDs "
              << cfg_.id_base << "," << cfg_.id_shoulder << "," << cfg_.id_elbow << ","
              << cfg_.id_wrist << ")\n";
    return true;
}

// ---------------------------------------------------------------------------
// Cinemática inversa: copia literal de pong_robot.hpp (computeIK de Valeria)
// ---------------------------------------------------------------------------
bool PongArm::computeIK(double x, double y, double gamma_rad,
                        double& q1, double& q2, double& q3) const {
    const double L1 = cfg_.L1, L2 = cfg_.L2, L3 = cfg_.L3;
    // x3 es lateral (-izq, +der) e y3 es frontal
    double x3 = x;
    double y3 = y - L3;

    double r2 = x3 * x3 + y3 * y3;
    double r = std::sqrt(r2);

    // Límite geométrico de los 2 primeros eslabones (L1 + L2)
    if (r > (L1 + L2 - 0.05) || r < std::abs(L1 - L2) + 0.1) return false;

    double cos_alpha = (L1 * L1 + L2 * L2 - r2) / (2.0 * L1 * L2);
    cos_alpha = std::clamp(cos_alpha, -1.0, 1.0);
    double alpha = std::acos(cos_alpha);

    // Flexión del codo (q2 = 0 cuando está extendido)
    q2 = -(M_PI - alpha);

    double sin_beta = (L2 * std::sin(alpha)) / r;
    sin_beta = std::clamp(sin_beta, -1.0, 1.0);
    double beta = std::asin(sin_beta);

    // Ángulo de la base continuo (-x3 para que girar a la izq sea positivo)
    double theta = std::atan2(-x3, y3);
    q1 = theta + beta;

    // Mantiene la pala paralela al eje x0
    q3 = gamma_rad - q1 - q2;
    return true;
}

bool PongArm::reachable(double x, double y) const {
    double q1, q2, q3;
    return computeIK(x, y, cfg_.gamma, q1, q2, q3);
}

double PongArm::maxReachableY(double x, double desired_y, double min_y) const {
    for (double y = desired_y; y >= min_y; y -= 0.1)
        if (reachable(x, y)) return y;
    return min_y;
}

int PongArm::jointToTicks(int joint, double rad) const {
    const double deg = rad * 180.0 / M_PI;
    int t = static_cast<int>(512 + cfg_.dir[joint] * deg * (1023.0 / 300.0)) + cfg_.offset[joint];
    return std::clamp(t, cfg_.tick_min[joint], cfg_.tick_max[joint]);
}

// ---------------------------------------------------------------------------
// Órdenes
// ---------------------------------------------------------------------------
void PongArm::setLateralTarget(double x) {
    target_x_ = std::clamp(x, cfg_.lateral_min, cfg_.lateral_max);
}

bool PongArm::triggerSmash() {
    if (!armed_ || smash_ != SmashState::IDLE) return false;
    smash_x_ = cur_x_;
    // El golpe va lo más adelante posible sin salirse del workspace en esta x
    // (esto evita el "desbordamiento" de Y=16.5 en los extremos).
    smash_strike_y_ = maxReachableY(smash_x_, cfg_.strike_y, cfg_.home_y);
    smash_ = SmashState::RECOIL;
    smash_t0_ = clock_();
    if (verbose()) std::cout << "[ARM " << cfg_.name << "] SMASH en x=" << smash_x_
              << " (strike y=" << smash_strike_y_ << ")\n";
    return true;
}

void PongArm::setArmed(bool armed) {
    armed_ = armed;
    if (!armed_) {
        smash_ = SmashState::IDLE;
        target_x_ = 0.0;
    }
}

void PongArm::goHome() {
    smash_ = SmashState::IDLE;
    target_x_ = 0.0;
    cmd_x_ = 0.0;
    send(0.0, cfg_.home_y, true);
}

bool PongArm::moveToPlanar(double x, double y) {
    cmd_x_ = target_x_ = x;
    return send(x, y, true);
}

bool PongArm::writeRaw(int id, int ticks) {
    if (simulate_ || !bus_) return true;
    const bool ok = bus_->write2(id, ADDR_GOAL_POSITION, std::clamp(ticks, 0, 1023));
    if (!ok) std::cerr << "[ARM " << cfg_.name << "] " << bus_->lastError() << "\n";
    return ok;
}

void PongArm::update() {
    const auto now = clock_();
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now - smash_t0_).count();

    // Movimiento lateral con velocidad máxima: evita saltos bruscos que hacen
    // que los XL-320 tiren de golpe mucha corriente y pierdan paquetes.
    const double goal_x = armed_ ? target_x_ : 0.0;
    const double dt = has_update_ ? std::chrono::duration<double>(now - last_update_).count() : 0.0;
    last_update_ = now;
    has_update_ = true;
    if (cfg_.max_lateral_speed > 0.0 && dt > 0.0) {
        const double step = cfg_.max_lateral_speed * std::min(dt, 0.1);
        cmd_x_ += std::clamp(goal_x - cmd_x_, -step, step);
    } else {
        cmd_x_ = goal_x;
    }
    double x = cmd_x_;
    double y = cfg_.home_y;

    switch (smash_) {
        case SmashState::IDLE:
            break;
        case SmashState::RECOIL:
            x = smash_x_;
            y = cfg_.recoil_y;
            if (ms >= cfg_.recoil_ms) { smash_ = SmashState::STRIKE; smash_t0_ = now; }
            break;
        case SmashState::STRIKE:
            x = smash_x_;
            y = smash_strike_y_;
            if (ms >= cfg_.strike_ms) { smash_ = SmashState::RETURN; smash_t0_ = now; }
            break;
        case SmashState::RETURN:
            x = smash_x_;
            if (ms >= cfg_.return_ms) { smash_ = SmashState::COOLDOWN; smash_t0_ = now; }
            break;
        case SmashState::COOLDOWN:
            if (ms >= cfg_.cooldown_ms) smash_ = SmashState::IDLE;
            break;
    }
    send(x, y, false);
}

bool PongArm::send(double x, double y, bool force) {
    double q1, q2, q3;
    if (!computeIK(x, y, cfg_.gamma, q1, q2, q3)) {
        // Fuera de alcance: nos quedamos donde estábamos (igual que antes).
        return false;
    }
    cur_x_ = x;
    cur_y_ = y;

    const std::array<int, 3> ticks = {jointToTicks(0, q1), jointToTicks(1, q2),
                                      jointToTicks(2, q3)};
    const auto now = clock_();
    const auto since = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_send_).count();
    if (!force && ticks == last_ticks_) return true;
    if (!force && since < cfg_.min_send_ms) return true;

    last_send_ = now;
    if (simulate_ || !bus_) {
        last_ticks_ = ticks;
        return true;
    }

    bool ok = true;
    // El hombro es fijo: se reenvía de vez en cuando por si alguien lo movió.
    const auto since_sh = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_shoulder_).count();
    if (force || since_sh > 500) {
        ok &= bus_->write2(cfg_.id_shoulder, ADDR_GOAL_POSITION, cfg_.shoulder_ticks);
        last_shoulder_ = now;
    }
    // Solo escribe los motores que cambiaron (menos tráfico en el bus).
    const int ids[3] = {cfg_.id_base, cfg_.id_elbow, cfg_.id_wrist};
    for (int j = 0; j < 3; ++j)
        if (force || ticks[j] != last_ticks_[j]) {
            if (bus_->write2(ids[j], ADDR_GOAL_POSITION, ticks[j])) {
                last_ticks_[j] = ticks[j];
            } else {
                ok = false;
                last_ticks_[j] = -1;  // solo se reintenta ESTE motor la próxima vez
            }
        }

    if (!ok) {
        ++send_errors_;
        if (std::chrono::duration_cast<std::chrono::seconds>(now - last_error_print_).count() >= 1) {
            std::cerr << "[ARM " << cfg_.name << "] Error de comunicación (" << send_errors_
                      << " en total): " << bus_->lastError() << "\n";
            last_error_print_ = now;
        }
    }
    return ok;
}

}  // namespace pongbot
