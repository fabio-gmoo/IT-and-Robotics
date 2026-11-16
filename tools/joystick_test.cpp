// =============================================================================
//  joystick_test: muestra lo que manda cada Arduino configurado en el YAML.
//    ./bin/joystick_test            (Ctrl+C para salir)
//    ./bin/joystick_test /dev/ttyACM1
// =============================================================================
#include <unistd.h>

#include <algorithm>
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "pongbot/config.hpp"
#include "pongbot/joystick.hpp"

using namespace pongbot;

int main(int argc, char** argv) {
    std::vector<std::string> devices;
    if (argc > 1) {
        for (int i = 1; i < argc; ++i) devices.push_back(argv[i]);
    } else {
        Config cfg;
        std::string err;
        if (!loadConfig("config/pongbot.yaml", cfg, err)) { std::cerr << err << "\n"; return 1; }
        for (const auto& j : cfg.joysticks)
            if (std::find(devices.begin(), devices.end(), j.device) == devices.end())
                devices.push_back(j.device);
    }

    std::vector<std::shared_ptr<SerialLineReader>> readers;
    for (const auto& d : devices) {
        std::string err;
        auto r = SerialLineReader::open(d, err);
        if (r) { readers.push_back(r); std::cout << "[OK] " << d << "\n"; }
        else std::cout << "[NO] " << err << "\n";
    }
    if (readers.empty()) return 1;

    std::cout << "Mueve los joysticks. Centro ~512, smash si Y > 800. Ctrl+C para salir.\n";
    while (true) {
        std::string line;
        for (auto& r : readers) {
            r->poll();
            line += r->device() + ": ";
            if (!r->hasData()) line += "(sin datos)   ";
            for (int v : r->values()) line += std::to_string(v) + " ";
            line += "  ";
        }
        std::cout << "\r" << line << "          " << std::flush;
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
