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

    // Conversión a ticks (512 = centro / 0 rad):
    int radToTicksBase(double rad) {
        double deg = rad * (180.0 / M_PI);
        return std::clamp(static_cast<int>(512 + (deg * (1023.0 / 300.0))), 0, 1023);
    }

    int radToTicksElbow(double rad) {
        double deg = rad * (180.0 / M_PI);
        return std::clamp(static_cast<int>(512 - (deg * (1023.0 / 300.0))), 0, 1023);
    }

    int radToTicksWrist(double rad) {
        double deg = rad * (180.0 / M_PI);
        return std::clamp(static_cast<int>(512 - (deg * (1023.0 / 300.0))), 0, 1023);
    }

    // Cinemática analítica continua para pala paralela (gamma = 0)
    bool computeIK(double x, double y, double gamma_rad, double &q1, double &q2, double &q3) {
        // En nuestro convenio, x3 es lateral (-izq, +der) e y3 es frontal
        // Con gamma = 0, la pala está paralela a la portería (eje x0)
        double x3 = x;
        double y3 = y - L3;

        double r2 = x3 * x3 + y3 * y3;
        double r = std::sqrt(r2);

        // Límite geométrico de los 2 primeros eslabones (L1 + L2 = 10.5 cm)
        if (r > (L1 + L2 - 0.05) || r < std::abs(L1 - L2) + 0.1) {
            return false;
        }

        double cos_alpha = (L1 * L1 + L2 * L2 - r2) / (2.0 * L1 * L2);
        cos_alpha = std::clamp(cos_alpha, -1.0, 1.0);
        double alpha = std::acos(cos_alpha);

        // Flexión del codo (q2 = 0 cuando está extendido)
        q2 = -(M_PI - alpha);

        double sin_beta = (L2 * std::sin(alpha)) / r;
        sin_beta = std::clamp(sin_beta, -1.0, 1.0);
        double beta = std::asin(sin_beta);

        // Ángulo de la base continuo (-x3 para que girar a la izq sea positivo)
        double theta = std::atan2(-x3, y3);
        q1 = theta + beta;

        // Compensación de orientación: mantiene la pala estrictamente paralela al eje x0
        q3 = gamma_rad - q1 - q2;

        return true;
    }

    bool moveToPlanar(double x, double y, double gamma_rad = 0.0, int fixed_shoulder = 341) {
        double q1, q2, q3;
        if (!computeIK(x, y, gamma_rad, q1, q2, q3)) return false;

        int t1 = radToTicksBase(q1);
        int t3 = radToTicksElbow(q2);
        int t4 = radToTicksWrist(q3);

        writePosition(id2, fixed_shoulder); usleep(10000);
        writePosition(id1, t1);             usleep(10000);
        writePosition(id3, t3);             usleep(10000);
        writePosition(id4, t4);             usleep(10000);
        return true;
    }

    ~PongRobot() {
        if (portHandler != nullptr) portHandler->closePort();
    }
};

#endif
