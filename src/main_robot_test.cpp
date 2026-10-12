#include "pong_robot.hpp"
#include <unistd.h>
#include <iostream>

int main() {
    PongRobot robot(5.0, 5.5, 6.3, 1, 2, 3, 4);
    if (!robot.init()) return -1;

    const int FIXED_SHOULDER = 341;
    const double GAMMA_PARALLEL = 0.0;

    std::cout << "\n>>> 1. CENTRO RECTO AL FRENTE (Extension total: Y = 16.8 cm)" << std::endl;
    robot.moveToPlanar(0.0, 16.8, GAMMA_PARALLEL, FIXED_SHOULDER);
    sleep(4);

    std::cout << "\n>>> 2. IZQUIERDA (X = -4.0 cm, Y = 13.0 cm) - Verificando pala" << std::endl;
    robot.moveToPlanar(-4.0, 13.0, GAMMA_PARALLEL, FIXED_SHOULDER);
    sleep(4);

    std::cout << "\n>>> 3. RETORNO AL CENTRO RECTO (Y = 16.8 cm)" << std::endl;
    robot.moveToPlanar(0.0, 16.8, GAMMA_PARALLEL, FIXED_SHOULDER);
    sleep(3);

    std::cout << "\n>>> 4. DERECHA (X = +4.0 cm, Y = 13.0 cm) - Verificando pala" << std::endl;
    robot.moveToPlanar(4.0, 13.0, GAMMA_PARALLEL, FIXED_SHOULDER);
    sleep(4);

    std::cout << "\n>>> 5. RETORNO FINAL AL CENTRO RECTO (Y = 16.8 cm)" << std::endl;
    robot.moveToPlanar(0.0, 16.8, GAMMA_PARALLEL, FIXED_SHOULDER);
    sleep(2);

    return 0;
}
