#include "pong_robot.hpp"
#include <unistd.h>
#include <iostream>

int main() {
    // Parámetros: L1, L2, L3 (en cm) e IDs reales (1, 2, 3, 4)
    PongRobot robot(10.0, 10.0, 5.0, 1, 2, 3, 4);

    if (!robot.init()) {
        std::cerr << "[-] Error al inicializar el robot." << std::endl;
        return -1;
    }

    const int FIXED_SHOULDER_TICKS = 341;

    std::cout << "[1] Bloqueando Motor 2 en posicion fija: " << FIXED_SHOULDER_TICKS << " ticks..." << std::endl;
    robot.writePosition(2, FIXED_SHOULDER_TICKS);
    sleep(2);

    std::cout << "\n[2] Ejecutando cinematica inversa a punto seguro central:" << std::endl;
    std::cout << "    Target: X = 15.0 cm, Y = 0.0 cm, Gamma = 0.0 rad" << std::endl;
    robot.moveToPlanar(15.0, 0.0, 0.0, FIXED_SHOULDER_TICKS);
    sleep(3);

    std::cout << "\n[3] Desplazamiento lateral suave hacia la izquierda:" << std::endl;
    std::cout << "    Target: X = 14.0 cm, Y = 4.0 cm, Gamma = 0.1 rad" << std::endl;
    robot.moveToPlanar(14.0, 4.0, 0.1, FIXED_SHOULDER_TICKS);
    sleep(3);

    std::cout << "\n[4] Regresando al centro..." << std::endl;
    robot.moveToPlanar(15.0, 0.0, 0.0, FIXED_SHOULDER_TICKS);
    sleep(2);

    std::cout << "\n[+] Prueba de cinematica inversa planar completada con exito." << std::endl;
    return 0;
}
