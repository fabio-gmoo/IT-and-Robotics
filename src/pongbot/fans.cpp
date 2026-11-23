#include "pongbot/fans.hpp"

#include <cmath>

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
    // IMPORTANTE: vaciar lo que manda el Arduino (líneas de los joysticks).
    // Si nadie lo lee (p.ej. en IA vs IA) el buffer USB se llena, el Arduino deja
    // de recibir las órdenes "F.." y su watchdog apaga los ventiladores.
    if (serial_) serial_->poll();

    int want = 0;
    if (cfg_.enabled && active && ball.valid) {
        const double vx = ball.vel_valid ? ball.vel.x : 0.0;   // cm/s a lo largo de la mesa
        const double speed = ball.vel_valid ? cv::norm(ball.vel) : 0.0;
        if (std::fabs(vx) < cfg_.stopped_speed) {
            // Casi parada: el ventilador de su mitad la empuja al otro lado
            want = 1 << (ball.pos.x < 0 ? cfg_.left_fan : cfg_.right_fan);
        } else if (speed < cfg_.slow_speed) {
            // Lenta: soplar desde detrás para que llegue con más velocidad
            want = 1 << (vx < 0 ? cfg_.right_fan : cfg_.left_fan);
        }
    }
    mask_ = want;

    // Se reenvía cada 0.3 s porque el Arduino apaga todo si no recibe nada en 1.5 s.
    if (serial_ && (want != sent_mask_ || t - last_send_t_ > 0.3)) {
        serial_->writeLine("F" + std::to_string(want));
        sent_mask_ = want;
        last_send_t_ = t;
    }
}

}  // namespace pongbot
