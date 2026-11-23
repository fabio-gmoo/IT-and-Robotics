// =============================================================================
//  Joystick Arduino por puerto serie (no bloqueante).
//  Formato de línea (115200 baudios): "x,y,btn\n" (sketch actual), "x,y\n",
//  "x1,y1,x2,y2\n" o "x1,y1,b1,x2,y2,b2\n"
//  Varios jugadores pueden compartir el mismo Arduino (channel 0 / 1).
// =============================================================================
#ifndef PONGBOT_JOYSTICK_HPP
#define PONGBOT_JOYSTICK_HPP

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace pongbot {

class SerialLineReader {
public:
    // Comparte el lector si dos jugadores usan el mismo dispositivo.
    static std::shared_ptr<SerialLineReader> open(const std::string& device, std::string& error);
    ~SerialLineReader();
    // Lee todo lo disponible y guarda los valores de la ÚLTIMA línea completa.
    void poll();
    const std::vector<int>& values() const { return values_; }
    bool hasData() const { return !values_.empty(); }
    const std::string& device() const { return device_; }

private:
    SerialLineReader() = default;
    int fd_ = -1;
    std::string device_;
    std::string buffer_;
    std::vector<int> values_;
    static std::map<std::string, std::weak_ptr<SerialLineReader>>& registry();
};

}  // namespace pongbot

#endif
