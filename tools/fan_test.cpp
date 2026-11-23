// =============================================================================
//  fan_test: probar los ventiladores de esquina SIN cámara ni brazos.
//    ./bin/fan_test          -> enciende 0, 1, 2, 3 uno a uno (3 s cada uno)
//    ./bin/fan_test 2        -> solo el ventilador 2 durante 5 s
//    ./bin/fan_test all      -> los 4 a la vez 3 s (prueba de la fuente de 12 V)
//  Usa fans.device de config/pongbot.yaml (o --device /dev/ttyACM0).
// =============================================================================
#include <chrono>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include "pongbot/config.hpp"
#include "pongbot/fans.hpp"
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

    if (what == "all") {
        std::cout << "Los 4 ventiladores 3 s\n";
        hold(*serial, 0xF, 3.0);
    } else if (what == "cycle") {
        for (int i = 0; i < 4; ++i) {
            std::cout << "FAN " << i << "  ->  esquina " << fanCornerName(i) << std::endl;
            hold(*serial, 1 << i, 3.0);
            hold(*serial, 0, 0.6);
        }
    } else {
        const int i = std::atoi(what.c_str());
        if (i < 0 || i > 3) { std::cerr << "Uso: fan_test [0|1|2|3|all]\n"; return 1; }
        std::cout << "FAN " << i << "  ->  esquina " << fanCornerName(i) << " (5 s)\n";
        hold(*serial, 1 << i, 5.0);
    }
    serial->writeLine("F0");
    std::cout << "Todos apagados.\n";
    return 0;
}
