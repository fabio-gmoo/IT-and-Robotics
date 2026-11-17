#include "pongbot/sim.hpp"

#include <cmath>

namespace pongbot {

SimTable::SimTable(const VisionConfig& cfg) : cfg_(cfg) { serve(); }

void SimTable::serve() {
    std::uniform_real_distribution<float> ang(-0.6f, 0.6f);
    std::uniform_int_distribution<int> dir(0, 1);
    std::uniform_real_distribution<float> spd(18.f, 30.f);
    const float a = ang(rng_);
    const float s = spd(rng_);
    const float d = dir(rng_) ? 1.f : -1.f;
    pos_ = cv::Point2f(0.f, 0.f);
    vel_ = cv::Point2f(d * s * std::cos(a), s * std::sin(a));
    visible_ = true;
    respawn_timer_ = 0.0;
}

void SimTable::step(double dt, const std::vector<const PongArm*>& arms) {
    if (respawn_timer_ > 0.0) {
        respawn_timer_ -= dt;
        if (respawn_timer_ <= 0.0) serve();
        return;
    }
    const cv::Point2f prev = pos_;
    pos_ += vel_ * static_cast<float>(dt);

    // Paredes laterales
    const float W = static_cast<float>(cfg_.table_half_width);
    if (pos_.y > W) { pos_.y = 2 * W - pos_.y; vel_.y = -vel_.y; }
    if (pos_.y < -W) { pos_.y = -2 * W - pos_.y; vel_.y = -vel_.y; }

    for (int side : {1, -1}) {
        const PongArm* arm = nullptr;
        for (const PongArm* a : arms)
            if (a && a->config().side == side) arm = a;

        const double u_prev = side * prev.x;
        const double u = side * pos_.x;
        if (arm) {
            const double pu = arm->paddleU();
            const int k = side > 0 ? 0 : 1;
            const double pu_prev = has_last_[k] ? last_paddle_u_[k] : pu;
            last_paddle_u_[k] = pu;
            has_last_[k] = true;
            // Hay contacto si la bola y la pala "se cruzan" en este paso
            // (la bola avanza hacia la pala o la pala barre a la bola en el smash).
            const double before = u_prev - pu_prev, after = u - pu;
            const bool crossing = (before <= 0.3 && after >= -0.3) &&
                                  (side * vel_.x > 0 || pu < pu_prev);
            const double offset = pos_.y - arm->paddleTableY();
            if (crossing && std::fabs(offset) <= paddle_half_width) {
                const float boost = arm->isSmashing() ? 1.35f : 1.0f;
                vel_.x = -vel_.x * boost;
                vel_.y += static_cast<float>(offset * 3.0);   // "efecto" según dónde pega
                pos_.x = static_cast<float>(side * (pu - 0.4));
                const float speed = cv::norm(vel_);
                if (speed > 70.f) vel_ *= 70.f / speed;
                continue;
            }
            if (u > arm->config().goal_u + 4.0) {  // gol: desaparece y se vuelve a sacar
                visible_ = false;
                respawn_timer_ = 1.0;
                return;
            }
        } else {
            // Sin brazo en ese extremo: hay una pared (modos de 1 brazo)
            const double wall = 31.0;  // pared virtual del lado sin brazo (modos de 1 brazo)
            if (u > wall && side * vel_.x > 0) {
                pos_.x = static_cast<float>(side * (2 * wall - u));
                vel_.x = -vel_.x;
            }
        }
    }
}

}  // namespace pongbot
