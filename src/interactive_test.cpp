#include "pong_robot.hpp"
#include <iostream>
#include <iomanip>
#include <algorithm>

// Función para calibrar el Motor 2 en el rango [-90° a +90°]
// 0° = 512 ticks (centro neutro)
// Invertimos el signo (-) para corregir el sentido de giro
int shoulderDegToTicks(double deg) {
    deg = std::clamp(deg, -90.0, 90.0);
    int ticks = static_cast<int>(512 - (deg * (1023.0 / 300.0)));
    return std::clamp(ticks, 0, 1023);
}

// Función relativa para los motores planares 1, 3 y 4 [-90° a +90°]
int relDegToTicks(double deg) {
    deg = std::clamp(deg, -90.0, 90.0);
    int ticks = static_cast<int>(512 + (deg * (1023.0 / 300.0)));
    return std::clamp(ticks, 0, 1023);
}

int main() {
    // Dimensiones en cm (mídelas en el robot) y los 4 IDs: 1, 2, 3, 4
    double L1 = 10.0, L2 = 10.0, L3 = 5.0;
    PongRobot robot(L1, L2, L3, 1, 2, 3, 4);

    if (!robot.init()) {
        std::cerr << "[-] Error al inicializar comunicación con el robot." << std::endl;
        return -1;
    }

    // Por defecto inicia en el centro (512 ticks / 0°)
    int fixed_shoulder = 512;
    robot.writePosition(2, fixed_shoulder);

    char modo;
    while (true) {
        std::cout << "\n==============================================" << std::endl;
        std::cout << "Selecciona modo:" << std::endl;
        std::cout << " [H] Calibrar Hombro (Motor 2) [-90 a +90 grados]" << std::endl;
        std::cout << " [A] Control Articular (Motores 1, 3, 4 en grados)" << std::endl;
        std::cout << " [C] Cinemática Inversa Planar (X, Y, Gamma)" << std::endl;
        std::cout << " [S] Salir" << std::endl;
        std::cout << "Opcion: ";
        std::cin >> modo;

        if (modo == 'S' || modo == 's') break;

        if (modo == 'H' || modo == 'h') {
            double deg;
            std::cout << "Introduce ángulo para Motor 2 en grados [-90 a 90]: ";
            std::cin >> deg;

            fixed_shoulder = shoulderDegToTicks(deg);
            std::cout << "[+] Angulo: " << deg << " deg -> Enviando " << fixed_shoulder << " ticks a Motor 2..." << std::endl;
            robot.writePosition(2, fixed_shoulder);
        }
        else if (modo == 'A' || modo == 'a') {
            double d1, d3, d4;
            std::cout << "Motor 1 (Base) [-90 a 90 deg]: "; std::cin >> d1;
            std::cout << "Motor 3 (Codo) [-90 a 90 deg]: "; std::cin >> d3;
            std::cout << "Motor 4 (Pala) [-90 a 90 deg]: "; std::cin >> d4;

            int t1 = relDegToTicks(d1);
            int t3 = relDegToTicks(d3);
            int t4 = relDegToTicks(d4);

            std::cout << "[+] Ticks -> M1: " << t1 << " | M3: " << t3 << " | M4: " << t4 << std::endl;
            robot.writePosition(1, t1);
            robot.writePosition(3, t3);
            robot.writePosition(4, t4);
        }
        else if (modo == 'C' || modo == 'c') {
            double x, y, gamma_deg;
            std::cout << "Introduce X deseado (cm): "; std::cin >> x;
            std::cout << "Introduce Y deseado (cm): "; std::cin >> y;
            std::cout << "Introduce Gamma (orientación pala en grados): "; std::cin >> gamma_deg;

            double gamma_rad = gamma_deg * (M_PI / 180.0);
            robot.moveToPlanar(x, y, gamma_rad, fixed_shoulder);
        }
    }

    return 0;
}
