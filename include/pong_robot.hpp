#ifndef PONG_ROBOT_HPP
#define PONG_ROBOT_HPP

#include <cmath>
#include <iostream>
#include <algorithm>
#include <unistd.h>
#include "dynamixel_sdk/dynamixel_sdk.h"

#define PROTOCOL_VERSION      2.0
#define BAUDRATE              1000000
#define DEVICENAME            "/dev/ttyUSB0"

#define ADDR_TORQUE_ENABLE    24
#define ADDR_GOAL_POSITION    30
#define TORQUE_ENABLE         1

class PongRobot {
private:
    double L1, L2, L3;
    int id1, id2, id3, id4;

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

public:
    PongRobot(double l1, double l2, double l3, int m1, int m2, int m3, int m4)
        : L1(l1), L2(l2), L3(l3), id1(m1), id2(m2), id3(m3), id4(m4) {
        portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
    }

    bool enableTorque(int id) {
        uint8_t dxl_error = 0;
        int comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        return (comm_result == COMM_SUCCESS && dxl_error == 0);
    }

    void writePosition(int id, int ticks) {
        ticks = std::clamp(ticks, 0, 1023);
        uint8_t dxl_error = 0;
        packetHandler->write2ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, (uint16_t)ticks, &dxl_error);
    }

    bool init() {
        if (!portHandler->openPort() || !portHandler->setBaudRate(BAUDRATE)) {
            std::cerr << "[-] Error abriendo puerto serial " << DEVICENAME << std::endl;
            return false;
        }

        enableTorque(id1); usleep(15000);
        enableTorque(id2); usleep(15000);
        enableTorque(id3); usleep(15000);
        enableTorque(id4); usleep(15000);

        std::cout << "[+] Robot inicializado correctamente." << std::endl;
        return true;
    }

    // Conversión a ticks:
    // Motor 1: Centro en 512 (+deg gira izquierda, -deg gira derecha)
    int radToTicksBase(double rad) {
        double deg = rad * (180.0 / M_PI);
        return std::clamp(static_cast<int>(512 + (deg * (1023.0 / 300.0))), 0, 1023);
    }

    // Motor 3: Eje invertido (-deg)
    int radToTicksElbow(double rad) {
        double deg = rad * (180.0 / M_PI);
        return std::clamp(static_cast<int>(512 - (deg * (1023.0 / 300.0))), 0, 1023);
    }

    // Motor 4: Eje invertido (-deg)
    int radToTicksWrist(double rad) {
        double deg = rad * (180.0 / M_PI);
        return std::clamp(static_cast<int>(512 - (deg * (1023.0 / 300.0))), 0, 1023);
    }

    // Cinemática Inversa simétrica referenciada a x0
    bool computeIK(double x, double y, double gamma_rad, double &q1, double &q2, double &q3) {
        // En extensión completa frontal (recto)
        double total_reach = L1 + L2 + L3;
        if (std::abs(x) < 0.2 && y >= (total_reach - 0.5)) {
            q1 = 0.0;
            q2 = 0.0;
            q3 = 0.0;
            return true;
        }

        // Posición de la articulación de la muñeca restando el avance de L3
        double x3 = x - L3 * std::sin(-gamma_rad);
        double y3 = y - L3 * std::cos(gamma_rad);

        double r2 = x3 * x3 + y3 * y3;
        double r = std::sqrt(r2);

        if (r > (L1 + L2) || r < std::abs(L1 - L2) || r < 0.1) {
            std::cerr << "[-] Coordenada inalcanzable (r = " << r << " cm)" << std::endl;
            return false;
        }

        double cos_alpha = (L1 * L1 + L2 * L2 - r2) / (2.0 * L1 * L2);
        cos_alpha = std::clamp(cos_alpha, -1.0, 1.0);
        double alpha = std::acos(cos_alpha);

        double sin_beta = (L2 * std::sin(M_PI - alpha)) / r;
        sin_beta = std::clamp(sin_beta, -1.0, 1.0);
        double beta = std::asin(sin_beta);

        double theta = std::atan2(-x3, y3);

        // Si vamos a la izquierda (x3 < 0, theta > 0), el codo flexiona en sentido coherente
        if (x3 <= 0) {
            q2 = -(M_PI - alpha);
            q1 = theta + beta;
        } else {
            q2 = (M_PI - alpha);
            q1 = theta - beta;
        }

        // Compensación para mantener la pala paralela al eje transversal x0
        q3 = gamma_rad - q1 - q2;

        return true;
    }

    bool moveToPlanar(double x, double y, double gamma_rad = 0.0, int fixed_shoulder = 341) {
        double q1, q2, q3;
        if (!computeIK(x, y, gamma_rad, q1, q2, q3)) return false;

        int t1 = radToTicksBase(q1);
        int t3 = radToTicksElbow(q2);
        int t4 = radToTicksWrist(q3);

        std::cout << "[Target (" << x << ", " << y << ")] -> Ticks: M1=" << t1 
                  << " | M3=" << t3 << " | M4=" << t4 << std::endl;

        writePosition(id2, fixed_shoulder); usleep(15000);
        writePosition(id1, t1);             usleep(15000);
        writePosition(id3, t3);             usleep(15000);
        writePosition(id4, t4);             usleep(15000);
        return true;
    }

    ~PongRobot() {
        if (portHandler != nullptr) portHandler->closePort();
    }
};

#endif
