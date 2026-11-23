// =============================================================================
//  FanController: decide qué ventilador de esquina encender según la bola.
//  - Solo uno a la vez (el de la esquina donde está la bola).
//  - La bola tiene que estar on_delay_s en la esquina (no salta por una bola
//    que solo pasa), se mantiene hold_s tras salir, máximo max_on_s seguidos.
//  - La lógica (update) no depende del hardware: se puede probar sin Arduino.
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

    // Abre el puerto del Arduino (compartido con los joysticks si es el mismo).
    bool connect(std::string& error);
    bool connected() const { return serial_ != nullptr; }

    // Esquina (0..3) donde está una posición de mesa, o -1.
    int cornerOf(const cv::Point2f& p) const;

    // Llamar en cada vuelta. active=false (menú, pausa) => todo apagado.
    void update(const BallState& ball, double t, bool active);
    // Fuerza una máscara durante 'seconds' (pruebas con teclado).
    void forceMask(int mask, double t, double seconds);
    void allOff();

    int mask() const { return mask_; }
    const FanConfig& config() const { return cfg_; }

private:
    void send(int mask, double t, bool force);

    FanConfig cfg_;
    std::shared_ptr<SerialLineReader> serial_;
    int mask_ = 0;
    int sent_mask_ = -1;
    double last_send_t_ = -1e9;

    int candidate_ = -1;          // esquina donde está la bola
    double candidate_since_ = 0;
    int on_fan_ = -1;             // ventilador encendido
    double on_since_ = 0;
    double last_in_zone_ = -1e9;
    double rest_until_ = -1e9;
    int forced_mask_ = 0;
    double forced_until_ = -1e9;
};

const char* fanCornerName(int i);

}  // namespace pongbot

#endif
