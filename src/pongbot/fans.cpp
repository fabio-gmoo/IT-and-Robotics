#include "pongbot/fans.hpp"

namespace pongbot {

bool FanController::connect(std::string& error) {
    serial_ = SerialLineReader::open(cfg_.device, error);
    if (serial_) allOff();
    return serial_ != nullptr;
}

void FanController::allOff() {
    mask_ = 0;
    if (serial_) serial_->writeLine("F0");
    sent_mask_ = 0;
}

void FanController::update(const BallState& ball, double t, bool active) {
    int want = 0;
    if (cfg_.enabled && active && ball.valid)
        want = 1 << (ball.pos.x < 0 ? cfg_.left_fan : cfg_.right_fan);
    mask_ = want;

    // Se reenvía cada 0.3 s porque el Arduino apaga todo si no recibe nada en 1.5 s.
    if (serial_ && (want != sent_mask_ || t - last_send_t_ > 0.3)) {
        serial_->writeLine("F" + std::to_string(want));
        sent_mask_ = want;
        last_send_t_ = t;
    }
}

}  // namespace pongbot
