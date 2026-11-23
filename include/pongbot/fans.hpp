// =============================================================================
//  Ventiladores: bola en la mitad izquierda -> ventilador izquierdo,
//                bola en la mitad derecha  -> ventilador derecho,
//                sin bola o fuera de partida -> apagados.
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
    // Llamar en cada vuelta del bucle. active = estamos en partida.
    void update(const BallState& ball, double t, bool active);
    void allOff();
    int mask() const { return mask_; }

private:
    FanConfig cfg_;
    std::shared_ptr<SerialLineReader> serial_;
    int mask_ = 0;
    int sent_mask_ = -1;
    double last_send_t_ = -1e9;
};

}  // namespace pongbot

#endif
