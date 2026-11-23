// =============================================================================
//  fan_test: probar los ventiladores de esquina SIN cámara ni brazos.
//    ./bin/fan_test          -> ventilador de A, luego el de B (3 s cada uno)
//    ./bin/fan_test A        -> solo el ventilador detrás del brazo A, 5 s
//    ./bin/fan_test B        -> solo el ventilador detrás del brazo B, 5 s
//    ./bin/fan_test 0..3     -> un bit concreto (0 = D5, 1 = D6, 2 = D9, 3 = D10)
//  Usa fans.device de config/pongbot.yaml (o --device /dev/ttyACM0).
// =============================================================================
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "pongbot/config.hpp"
#include "pongbot/joystick.hpp"

using namespace pongbot;

static void hold(SerialLineReader& s, int mask, double seconds) {
    const auto t0 = std::chrono::steady_clock::now();
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() < seconds) {
        s.writeLine("F" + std::to_string(mask));   // reenvío: el Arduino apaga a los 1.5 s
        s.poll();                                  // vaciar lo que mandan los joysticks
        std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
}

int main(int argc, char** argv) {
    Config cfg;
    std::string err;
    if (!loadConfig("config/pongbot.yaml", cfg, err)) { std::cerr << err << "\n"; return 1; }
    std::string device = cfg.fans.device, what = "cycle";
    for (int i = 1; i < argc; ++i) {
        if (!std::strcmp(argv[i], "--device") && i + 1 < argc) device = argv[++i];
        else what = argv[i];
    }
    auto serial = SerialLineReader::open(device, err);
    if (!serial) { std::cerr << "[-] " << err << "\n"; return 1; }
    std::cout << "[OK] " << device << "\n    Esperando 2 s (el Arduino se reinicia al abrir el puerto)...\n";
    std::this_thread::sleep_for(std::chrono::seconds(2));

    const int bitA = cfg.fans.fan_pos_x, bitB = cfg.fans.fan_neg_x;
    const char* pin[4] = {"D5", "D6", "D9", "D10"};
    auto show = [&](const char* name, int bit, double secs) {
        std::cout << "Ventilador " << name << " (pin " << pin[bit & 3] << ") " << secs << " s" << std::endl;
        hold(*serial, 1 << bit, secs);
        hold(*serial, 0, 0.8);
    };
    if (what == "cycle") {
        show("detrás del brazo A", bitA, 3.0);
        show("detrás del brazo B", bitB, 3.0);
    } else if (what == "A" || what == "a") {
        show("detrás del brazo A", bitA, 5.0);
    } else if (what == "B" || what == "b") {
        show("detrás del brazo B", bitB, 5.0);
    } else {
        const int i = std::atoi(what.c_str());
        if (i < 0 || i > 3) { std::cerr << "Uso: fan_test [A|B|0|1|2|3]\n"; return 1; }
        show("bit", i, 5.0);
    }
    serial->writeLine("F0");
    std::cout << "Todos apagados.\n";
    return 0;
}
