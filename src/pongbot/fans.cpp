#include "pongbot/fans.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>

namespace pongbot {

const char* fanCornerName(int i) {
    switch (i) {
        case 0: return "+X +Y (arm A side, top in the top-down view)";
        case 1: return "+X -Y (arm A side, bottom)";
        case 2: return "-X +Y (arm B side, top)";
        case 3: return "-X -Y (arm B side, bottom)";
    }
    return "?";
}

bool FanController::connect(std::string& error) {
    serial_ = SerialLineReader::open(cfg_.device, error);
    if (serial_) allOff();
    return serial_ != nullptr;
}

int FanController::cornerOf(const cv::Point2f& p) const {
    const double ax = std::fabs(p.x), ay = std::fabs(p.y);
    if (ax < cfg_.corner_x || ax > cfg_.max_x || ay < cfg_.corner_y) return -1;
    const bool px = p.x > 0, py = p.y > 0;
    return px ? (py ? 0 : 1) : (py ? 2 : 3);
}

void FanController::forceMask(int mask, double t, double seconds) {
    forced_mask_ = mask & 0xF;
    forced_until_ = t + seconds;
}

void FanController::allOff() {
    on_fan_ = -1;
    candidate_ = -1;
    mask_ = 0;
    if (serial_) {
        serial_->writeLine("F0");
        sent_mask_ = 0;
    }
}

void FanController::update(const BallState& ball, double t, bool active) {
    int want = 0;

    if (t < forced_until_) {
        want = forced_mask_;
    } else if (active && cfg_.enabled) {
        const int corner = ball.valid ? cornerOf(ball.pos) : -1;

        // ¿Cuánto tiempo lleva la bola en la misma esquina?
        if (corner != candidate_) {
            candidate_ = corner;
            candidate_since_ = t;
        }
        if (corner >= 0) last_in_zone_ = t;

        if (on_fan_ >= 0) {
            // Se apaga si la bola pasó a OTRA esquina, o si lleva hold_s fuera de las esquinas.
            const bool left_zone = corner != on_fan_ &&
                                   (corner >= 0 || t - last_in_zone_ > cfg_.hold_s);
            const bool too_long = t - on_since_ > cfg_.max_on_s;
            if (left_zone || too_long) {
                if (too_long) rest_until_ = t + cfg_.rest_s;
                on_fan_ = -1;
            }
        }
        if (on_fan_ < 0 && corner >= 0 && t >= rest_until_ &&
            t - candidate_since_ >= cfg_.on_delay_s) {
            on_fan_ = corner;
            on_since_ = t;
        }
        want = on_fan_ >= 0 ? (1 << on_fan_) : 0;
    } else {
        on_fan_ = -1;
        candidate_ = -1;
    }

    mask_ = want;
    send(want, t, false);
}

void FanController::send(int mask, double t, bool force) {
    if (!serial_) return;
    // Reenvío periódico: el Arduino apaga todo si no recibe nada en 1.5 s.
    if (!force && mask == sent_mask_ && t - last_send_t_ < cfg_.refresh_s) return;
    serial_->writeLine("F" + std::to_string(mask));
    sent_mask_ = mask;
    last_send_t_ = t;
}

}  // namespace pongbot
