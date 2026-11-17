#include "pongbot/controllers.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace pongbot {

// ============================================================================
// IA
// ============================================================================
void AIController::update(PongArm& arm, const TableVision& vision, double /*dt*/) {
    const ArmConfig& c = arm.config();
    const BallState& b = vision.ball();
    if (!b.valid) {
        status_ = "sin bola";
        return;  // mantiene la última posición
    }

    const double u = c.side * b.pos.x;            // distancia "hacia" este robot
    const double vu = b.vel_valid ? c.side * b.vel.x : 0.0;  // >0: se acerca

    // 1) Lateral: si la bola viene hacia nosotros, ir a donde VA a llegar;
    //    si no, seguir su Y actual (lo que ya funcionaba).
    double tableY = b.pos.y;
    double y_pred = 0.0, t_pred = 0.0;
    const bool predicted = predict_ && vision.predictIntercept(c.side, c.paddle_u, y_pred, t_pred);
    if (predicted) tableY = y_pred;
    arm.setLateralTarget(c.lateral_sign * (tableY - c.lateral_center));

    // 2) Smash
    const bool lateral_ok = std::fabs(b.pos.y - arm.paddleTableY()) <= c.smash_lateral_window;
    bool fire = false;
    if (lateral_ok && u < c.goal_u) {
        if (vu > c.smash_min_speed) {
            // Cruce del umbral, o llegará al umbral antes de smash_lead_s
            // (a 30 FPS una bola rápida se "salta" una ventana fija).
            const double time_to_trigger = (c.smash_trigger_u - u) / vu;
            fire = (u >= c.smash_trigger_u) || (time_to_trigger <= c.smash_lead_s);
        } else if (std::fabs(vu) < 2.0 && u >= c.smash_trigger_u - 0.5 &&
                   u <= c.paddle_u + 0.5) {
            // Bola casi parada delante de la pala: empujarla, pero como mucho
            // 2 veces seguidas si no se ha movido (bola atascada / fuera de alcance).
            if (cv::norm(b.pos - still_pos_) > 1.5f) still_smashes_ = 0;
            if (still_smashes_ < 2 && arm.smashState() == PongArm::SmashState::IDLE) {
                fire = true;
                ++still_smashes_;
                still_pos_ = b.pos;
            }
        }
    }
    if (std::fabs(vu) >= 2.0) still_smashes_ = 0;
    if (fire) arm.triggerSmash();

    char buf[96];
    std::snprintf(buf, sizeof(buf), "u=%.1f vu=%.0f%s", u, vu, predicted ? " pred" : "");
    status_ = buf;
}

// ============================================================================
// Joystick / teclado
// ============================================================================
JoystickController::JoystickController(const JoystickConfig& cfg, KeyBinding keys)
    : cfg_(cfg), keys_(keys) {
    if (!cfg_.device.empty() && cfg_.device != "none")
        reader_ = SerialLineReader::open(cfg_.device, open_error_);
}

void JoystickController::handleKey(int key) {
    if (key == keys_.left) key_dx_ -= 0.8;
    else if (key == keys_.right) key_dx_ += 0.8;
    else if (key == keys_.smash) key_smash_ = true;
}

void JoystickController::update(PongArm& arm, const TableVision& /*vision*/, double dt) {
    const double xmin = arm.config().lateral_min, xmax = arm.config().lateral_max;
    bool smash_now = false;

    if (reader_) {
        reader_->poll();
        const auto& v = reader_->values();
        // Formatos aceptados por línea:
        //   "x,y" / "x1,y1,x2,y2"            -> 2 valores por stick (sin botón)
        //   "x,y,btn" / "x1,y1,b1,x2,y2,b2"  -> 3 valores por stick (sketch de Valeria)
        const size_t stride = (v.size() % 3 == 0) ? 3 : 2;
        const size_t ix = stride * static_cast<size_t>(cfg_.channel);
        if (v.size() >= ix + 2) {
            const int raw_x = v[ix];
            const int raw_y = v[ix + 1];
            // Botón con INPUT_PULLUP: 0 = pulsado
            const bool button = stride == 3 && v.size() >= ix + 3 && v[ix + 2] == 0;
            // Misma zona muerta y escala que joystick_teleop.cpp
            double vx = 0.0;
            if (raw_x > cfg_.deadzone_high) vx = (raw_x - cfg_.deadzone_high) / 450.0;
            else if (raw_x < cfg_.deadzone_low) vx = (raw_x - cfg_.deadzone_low) / 450.0;
            vx = std::clamp(vx, -1.0, 1.0);
            if (cfg_.invert_x) vx = -vx;
            x_ += vx * cfg_.max_speed * dt;

            // Smash por flanco: empujar hacia delante y soltar para repetir
            const bool pushed = (cfg_.smash_threshold > 0 && raw_y > cfg_.smash_threshold) || button;
            if (pushed && !smash_latch_) smash_now = true;
            smash_latch_ = pushed;

            char buf[64];
            std::snprintf(buf, sizeof(buf), "raw=(%d,%d)%s", raw_x, raw_y, button ? " BTN" : "");
            status_ = buf;
        } else {
            status_ = "esperando datos del Arduino";
        }
    } else {
        status_ = "teclado";
    }

    // Teclado (siempre activo; útil si el Arduino falla)
    x_ += key_dx_;
    key_dx_ = 0.0;
    if (key_smash_) smash_now = true;
    key_smash_ = false;

    x_ = std::clamp(x_, xmin, xmax);
    arm.setLateralTarget(x_);
    if (smash_now) arm.triggerSmash();
}

}  // namespace pongbot
