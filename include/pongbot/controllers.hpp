// =============================================================================
//  Controladores: deciden la x lateral y cuándo hacer smash para UN brazo.
//    - AIController:       sigue / predice la bola y golpea cuando llega.
//    - JoystickController: joystick Arduino (y teclado como respaldo).
// =============================================================================
#ifndef PONGBOT_CONTROLLERS_HPP
#define PONGBOT_CONTROLLERS_HPP

#include <memory>
#include <string>

#include "pongbot/arm.hpp"
#include "pongbot/config.hpp"
#include "pongbot/joystick.hpp"
#include "pongbot/vision.hpp"

namespace pongbot {

class Controller {
public:
    virtual ~Controller() = default;
    // dt en segundos desde la última llamada
    virtual void update(PongArm& arm, const TableVision& vision, double dt) = 0;
    virtual void handleKey(int /*key*/) {}
    virtual std::string label() const = 0;   // "IA", "JOY", "TECLADO"
    virtual std::string status() const { return ""; }
};

class AIController : public Controller {
public:
    explicit AIController(bool predict) : predict_(predict) {}
    void update(PongArm& arm, const TableVision& vision, double dt) override;
    std::string label() const override { return "IA"; }
    std::string status() const override { return status_; }

private:
    bool predict_;
    std::string status_;
    // Para no golpear sin parar una bola atascada (p.ej. en una esquina):
    int still_smashes_ = 0;
    cv::Point2f still_pos_;
};

struct KeyBinding {
    int left, right, smash;
};

class JoystickController : public Controller {
public:
    // Si el Arduino no abre, funciona solo con teclado (keys).
    JoystickController(const JoystickConfig& cfg, KeyBinding keys);
    void update(PongArm& arm, const TableVision& vision, double dt) override;
    void handleKey(int key) override;
    std::string label() const override { return reader_ ? "JOY" : "TECLADO"; }
    std::string status() const override { return status_; }
    bool hasJoystick() const { return reader_ != nullptr; }
    const std::string& openError() const { return open_error_; }

private:
    JoystickConfig cfg_;
    KeyBinding keys_;
    std::shared_ptr<SerialLineReader> reader_;
    std::string open_error_;
    double x_ = 0.0;
    bool smash_latch_ = false;
    double key_dx_ = 0.0;
    bool key_smash_ = false;
    std::string status_;
};

}  // namespace pongbot

#endif
