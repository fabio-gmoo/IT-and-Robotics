// =============================================================================
//  Simulador muy simple de la mesa (modo --sim): bola con rebotes en las
//  paredes y en las palas de los brazos simulados. Sirve para probar la
//  lógica de la IA, el joystick, los goles y la GUI SIN hardware.
// =============================================================================
#ifndef PONGBOT_SIM_HPP
#define PONGBOT_SIM_HPP

#include <random>
#include <vector>

#include <opencv2/core.hpp>

#include "pongbot/arm.hpp"
#include "pongbot/config.hpp"

namespace pongbot {

class SimTable {
public:
    explicit SimTable(const VisionConfig& cfg);
    // arms: los brazos que están jugando (su side indica en qué extremo están)
    void step(double dt, const std::vector<const PongArm*>& arms);
    void serve();
    bool visible() const { return visible_; }
    cv::Point2f pos() const { return pos_; }

    double paddle_half_width = 2.5;  // cm (aprox. medio ancho de la pala)

private:
    VisionConfig cfg_;
    cv::Point2f pos_, vel_;
    bool visible_ = true;
    double respawn_timer_ = 0.0;
    std::mt19937 rng_{12345};
    double last_paddle_u_[2] = {0.0, 0.0};   // [0]: side +1, [1]: side -1
    bool has_last_[2] = {false, false};
};

}  // namespace pongbot

#endif
