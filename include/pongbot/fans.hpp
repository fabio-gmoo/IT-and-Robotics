// =============================================================================
//  FanController: un ventilador detrás de cada brazo.
//  Si la bola está TRANCADA en la zona central (se ha movido menos de
//  stuck_radius en stuck_time_s), enciende el ventilador del lado donde está:
//    X > 0 (mitad del brazo A) -> ventilador detrás de A (sopla hacia B)
//    X < 0 (mitad del brazo B) -> ventilador detrás de B (sopla hacia A)
//  Nunca los dos a la vez. La lógica no depende del hardware (se prueba en make test).
// =============================================================================
#ifndef PONGBOT_FANS_HPP
#define PONGBOT_FANS_HPP

#include <memory>
#include <string>

#include "pongbot/config.hpp"
#include "pongbot/joystick.hpp"
#include "pongbot/vision.hpp"

namespace pongbot {

class FanController {
public:
    explicit FanController(const FanConfig& cfg) : cfg_(cfg) {}

    bool connect(std::string& error);
    bool connected() const { return serial_ != nullptr; }

    // Llamar en cada vuelta. active=false (menú) => todo apagado.
    void update(const BallState& ball, double t, bool active);
    void forceMask(int mask, double t, double seconds);
    void allOff();

    int mask() const { return mask_; }
    bool stuck() const { return stuck_; }
    double stuckFor(double t) const { return anchor_valid_ ? t - anchor_t_ : 0.0; }
    const FanConfig& config() const { return cfg_; }

private:
    void send(int mask, double t, bool force);

    FanConfig cfg_;
    std::shared_ptr<SerialLineReader> serial_;
    int mask_ = 0;
    int sent_mask_ = -1;
    double last_send_t_ = -1e9;

    // Detección de bola trancada
    bool anchor_valid_ = false;
    cv::Point2f anchor_;
    double anchor_t_ = 0.0;
    double last_seen_t_ = -1e9;
    bool stuck_ = false;

    int on_bit_ = -1;
    double on_since_ = 0.0;
    double rest_until_ = -1e9;
    int forced_mask_ = 0;
    double forced_until_ = -1e9;
};

}  // namespace pongbot

#endif
