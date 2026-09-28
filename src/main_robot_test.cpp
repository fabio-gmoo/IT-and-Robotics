#include "pong_robot.hpp"
#include <unistd.h>
#include <iostream>

int main() {
    // Dimensiones en cm: L1 = 5.0, L2 = 5.5, L3 = 6.3
    // Motores: 1 (Base), 2 (Hombro vertical), 3 (Codo), 4 (Pala)
    PongRobot robot(5.0, 5.5, 6.3, 1, 2, 3, 4);

    if (!robot.init()) {
        std::cerr << "[-] Error al inicializar el robot." << std::endl;
        return -1;
    }

    const int FIXED_SHOULDER = 341;
    // Gamma = 0.0 rad asegura que la pala permanezca perpendicular a la mesa
    const double GAMMA_FIXED = 0.0;

    std::cout << "\n=======================================================" << std::endl;
    std::cout << ">>> 1. BLOQUEANDO HOMBRO (M2 en 341) Y POSTURA RECTA" << std::endl;
    std::cout << "=======================================================" << std::endl;
    robot.writePosition(2, FIXED_SHOULDER);
    robot.writePosition(1, 512);
    robot.writePosition(3, 512);
    robot.writePosition(4, 512);
    std::cout << "[+] Brazo centrado. Esperando 3 segundos..." << std::endl;
    sleep(3);

    std::cout << "\n>>> 2. ESPERA DE PELOTA: Centro medio (X = 0.0 cm, Y = 13.0 cm)" << std::endl;
    std::cout << "       (El codo y la base se flexionan, la pala queda recta al frente)" << std::endl;
    robot.moveToPlanar(0.0, 13.0, GAMMA_FIXED, FIXED_SHOULDER);
    sleep(3);

    std::cout << "\n>>> 3. BLOQUEO A LA IZQUIERDA (X = -4.0 cm, Y = 12.0 cm)" << std::endl;
    std::cout << "       (El brazo se desplaza a la izq. y la pala sigue apuntando al frente)" << std::endl;
    robot.moveToPlanar(-4.0, 12.0, GAMMA_FIXED, FIXED_SHOULDER);
    sleep(3);

    std::cout << "\n>>> 4. RETORNO AL CENTRO DE ESPERA (X = 0.0 cm, Y = 13.0 cm)" << std::endl;
    robot.moveToPlanar(0.0, 13.0, GAMMA_FIXED, FIXED_SHOULDER);
    sleep(3);

    std::cout << "\n>>> 5. BLOQUEO A LA DERECHA (X = +4.0 cm, Y = 12.0 cm)" << std::endl;
    std::cout << "       (El brazo cruza a la der. y la pala mantiene el plano de golpeo)" << std::endl;
    robot.moveToPlanar(4.0, 12.0, GAMMA_FIXED, FIXED_SHOULDER);
    sleep(3);

    std::cout << "\n>>> 6. REMATE FRONTAL AVANZADO (X = 0.0 cm, Y = 15.0 cm)" << std::endl;
    robot.moveToPlanar(0.0, 15.0, GAMMA_FIXED, FIXED_SHOULDER);
    sleep(3);

    std::cout << "\n>>> 7. RETORNO FINAL A REPOSO TOTAL AL FRENTE (X = 0.0, Y = 16.8 cm)" << std::endl;
    robot.moveToPlanar(0.0, 16.8, GAMMA_FIXED, FIXED_SHOULDER);
    sleep(2);

    std::cout << "\n[+] Prueba de intercepción con pala fija completada exitosamente." << std::endl;
    return 0;
}
