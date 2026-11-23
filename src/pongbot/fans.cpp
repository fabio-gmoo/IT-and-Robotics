#include "pongbot/fans.hpp"

#include <cmath>

namespace pongbot {

bool FanController::connect(std::string& error) {
    serial_ = SerialLineReader::open(cfg_.device, error);
    if (serial_) allOff();
    return serial_ != nullptr;
}

void FanController::forceMask(int mask, double t, double seconds) {
    forced_mask_ = mask & 0xF;
    forced_until_ = t + seconds;
}

void FanController::allOff() {
    on_bit_ = -1;
    mask_ = 0;
    anchor_valid_ = false;
    stuck_ = false;
    if (serial_) {
        serial_->writeLine("F0");
        sent_mask_ = 0;
    }
}

void FanController::update(const BallState& ball, double t, bool active) {
    int want = 0;

    if (t < forced_until_) {
        want = forced_mask_;
    } else if (!active || !cfg_.enabled) {
        on_bit_ = -1;
        anchor_valid_ = false;
        stuck_ = false;
    } else {
        // 1) ¿Bola trancada en la zona central?
        if (ball.valid) {
            last_seen_t_ = t;
            const bool in_middle = std::fabs(ball.pos.x) <= cfg_.middle_x;
            if (!in_middle) {
                anchor_valid_ = false;  // cerca de una pala: se encarga el brazo
            } else if (!anchor_valid_ || cv::norm(ball.pos - anchor_) > cfg_.stuck_radius) {
                anchor_ = ball.pos;     // se ha movido: reiniciamos la cuenta
                anchor_t_ = t;
                anchor_valid_ = true;
            }
        } else if (t - last_seen_t_ > 0.5) {
            anchor_valid_ = false;      // sin bola => nada
        }
        stuck_ = anchor_valid_ && (t - anchor_t_) >= cfg_.stuck_time_s;

        // 2) Encender / apagar
        if (on_bit_ >= 0) {
            const double on_for = t - on_since_;
            if (on_for >= cfg_.max_on_s) {          // no consigue moverla: descansar
                on_bit_ = -1;
                rest_until_ = t + cfg_.rest_s;
            } else if (on_for >= cfg_.min_on_s && !stuck_) {
                on_bit_ = -1;                       // ya se mueve
            }
        }
        if (on_bit_ < 0 && stuck_ && t >= rest_until_) {
            on_bit_ = (anchor_.x >= 0.f) ? cfg_.fan_pos_x : cfg_.fan_neg_x;
            on_since_ = t;
        }
        want = (on_bit_ >= 0) ? (1 << on_bit_) : 0;
    }

    mask_ = want;
    send(want, t, false);
}

void FanController::send(int mask, double t, bool force) {
    if (!serial_) return;
    if (!force && mask == sent_mask_ && t - last_send_t_ < cfg_.refresh_s) return;
    serial_->writeLine("F" + std::to_string(mask));
    sent_mask_ = mask;
    last_send_t_ = t;
}

}  // namespace pongbot
