// fan_test: probar los ventiladores sin cámara ni brazos.
//   ./bin/fan_test         -> derecho 3 s, luego izquierdo 3 s
//   ./bin/fan_test R | L   -> solo el derecho / izquierdo, 5 s
#include <chrono>
#include <iostream>
#include <string>
#include <thread>

#include "pongbot/config.hpp"
#include "pongbot/joystick.hpp"

using namespace pongbot;

static void hold(SerialLineReader& s, int mask, double seconds) {
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
        s.writeLine("F" + std::to_string(mask));
        s.poll();
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}

int main(int argc, char** argv) {
    Config cfg;
    std::string err;
    if (!loadConfig("config/pongbot.yaml", cfg, err)) { std::cerr << err << "\n"; return 1; }
    auto serial = SerialLineReader::open(cfg.fans.device, err);
    if (!serial) { std::cerr << "[-] " << err << "\n"; return 1; }
    std::cout << "[OK] " << cfg.fans.device << " (esperando 2 s a que arranque el Arduino)\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    const std::string what = argc > 1 ? argv[1] : "";
    const int R = 1 << cfg.fans.right_fan, L = 1 << cfg.fans.left_fan;
    if (what != "L" && what != "l") { std::cout << "Ventilador DERECHO\n" << std::flush; hold(*serial, R, what.empty() ? 3 : 5); hold(*serial, 0, 0.8); }
    if (what != "R" && what != "r") { std::cout << "Ventilador IZQUIERDO\n" << std::flush; hold(*serial, L, what.empty() ? 3 : 5); }
    serial->writeLine("F0");
    std::cout << "Apagados.\n";
    return 0;
}
