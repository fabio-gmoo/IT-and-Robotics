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

#define ADDR_TORQUE_ENABLE    24   // 1 byte
#define ADDR_GOAL_POSITION    30   // 2 bytes (0 a 1023)
#define TORQUE_ENABLE         1
#define TORQUE_DISABLE        0

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
        int dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << "[-] Error al activar torque en ID " << id << ": " 
                      << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
            return false;
        }
        std::cout << "[+] Torque activado en ID: " << id << std::endl;
        return true;
    }

    void writePosition(int id, int ticks) {
        ticks = std::clamp(ticks, 0, 1023);
        uint8_t dxl_error = 0;
        int dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, (uint16_t)ticks, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            std::cerr << "[-] Fallo enviando posicion a ID " << id << ": " 
                      << packetHandler->getTxRxResult(dxl_comm_result) << std::endl;
        }
    }

    bool init() {
        if (!portHandler->openPort()) {
            std::cerr << "[-] No se pudo abrir el puerto: " << DEVICENAME << std::endl;
            return false;
        }
        if (!portHandler->setBaudRate(BAUDRATE)) {
            std::cerr << "[-] No se pudo configurar baudrate" << std::endl;
            return false;
        }

        enableTorque(id1); usleep(20000);
        enableTorque(id2); usleep(20000);
        enableTorque(id3); usleep(20000);
        enableTorque(id4); usleep(20000);

        std::cout << "[+] Los 4 motores han sido inicializados." << std::endl;
        return true;
    }

    // Cinemática Inversa planar (guía PongBot) asignada a ID1 (base), ID3 (codo) e ID4 (pala)
    bool computeIK(double xe, double ye, double gamma_rad, double &q1, double &q2, double &q3) {
        double x3 = xe - L3 * std::cos(gamma_rad);
        double y3 = ye - L3 * std::sin(gamma_rad);

        double r2 = x3 * x3 + y3 * y3;
        double cos_alpha = (r2 - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

        if (cos_alpha < -1.0 || cos_alpha > 1.0) {
            std::cerr << "[-] Posición fuera de rango de trabajo planar." << std::endl;
            return false;
        }

        double alpha = std::acos(cos_alpha);
        double beta = std::asin(std::clamp((L2 * std::sin(alpha)) / std::sqrt(r2), -1.0, 1.0));

        q1 = std::atan2(y3, x3) + beta;
        q2 = -(M_PI - alpha);
        q3 = gamma_rad - q1 - q2;

        return true;
    }

    int radToTicks(double rad) {
        double deg = rad * (180.0 / M_PI);
        int ticks = static_cast<int>(512 + (deg * (1023.0 / 300.0)));
        return std::clamp(ticks, 0, 1023);
    }

    // Movimiento planar: motor 2 bloqueado en altura fija
    void moveToPlanar(double xe, double ye, double gamma_rad, int fixed_shoulder_ticks = 512) {
        double q1, q3, q4;
        if (computeIK(xe, ye, gamma_rad, q1, q3, q4)) {
            writePosition(id1, radToTicks(q1));
            writePosition(id2, fixed_shoulder_ticks); // Motor 2 se queda estático
            writePosition(id3, radToTicks(q3));
            writePosition(id4, radToTicks(q4));
            std::cout << "[+] Movimiento Planar -> Base(ID1): " << radToTicks(q1)
                      << " | Codo(ID3): " << radToTicks(q3)
                      << " | Pala(ID4): " << radToTicks(q4) << std::endl;
        }
    }

    ~PongRobot() {
        if (portHandler != nullptr) {
            portHandler->closePort();
        }
    }
};

#endif
