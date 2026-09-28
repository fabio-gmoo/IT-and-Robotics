#include "pong_robot.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>

#define ARDUINO_PORT "/dev/ttyACM0" // Ajusta a /dev/ttyUSB1 si fuera necesario

int openSerial(const char* portName) {
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) return -1;

    struct termios options;
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);

    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    options.c_oflag &= ~OPOST;

    tcsetattr(fd, TCSANOW, &options);
    return fd;
}

int main() {
    // Dimensiones en cm: L1=5.0, L2=5.5, L3=6.3
    PongRobot robot(5.0, 5.5, 6.3, 1, 2, 3, 4);
    if (!robot.init()) {
        std::cerr << "[-] Error al iniciar Dynamixel." << std::endl;
        return -1;
    }

    int serial_fd = openSerial(ARDUINO_PORT);
    if (serial_fd == -1) {
        std::cerr << "[-] No se pudo abrir el puerto del Arduino (" << ARDUINO_PORT << ")." << std::endl;
        return -1;
    }

    const int FIXED_SHOULDER = 341;
    const double GAMMA_PARALLEL = 0.0;

    // Posición inicial de reposo exacta (recto al frente comprobado)
    double current_x = 0.0;
    double current_y = 16.8;

    std::cout << "[+] Colocando en posición inicial recta (X=0.0, Y=16.8)..." << std::endl;
    robot.moveToPlanar(current_x, current_y, GAMMA_PARALLEL, FIXED_SHOULDER);
    sleep(2);

    std::cout << "\n==============================================" << std::endl;
    std::cout << "  CONTROL INCREMENTAL POR JOYSTICK PONGBOT    " << std::endl;
    std::cout << "  - Centro neutro: Sin movimiento             " << std::endl;
    std::cout << "  - Pulsa el botón del joystick para centrar  " << std::endl;
    std::cout << "==============================================" << std::endl;

    std::string buffer = "";
    char ch;
    int loop_counter = 0;

    while (true) {
        if (read(serial_fd, &ch, 1) > 0) {
            if (ch == '\n') {
                std::stringstream ss(buffer);
                std::string s_x, s_y, s_btn;
                if (std::getline(ss, s_x, ',') && std::getline(ss, s_y, ',') && std::getline(ss, s_btn)) {
                    try {
                        int raw_x = std::stoi(s_x);
                        int raw_y = std::stoi(s_y);
                        int btn   = std::stoi(s_btn);

                        // Si presionas el botón del joystick (activo en 0), regresa al centro recto
                        if (btn == 0) {
                            current_x = 0.0;
                            current_y = 16.8;
                        } else {
                            // Zona muerta (Deadzone): ignorar variaciones menores alrededor de 512
                            double joy_dx = 0.0;
                            double joy_dy = 0.0;

                            if (raw_x > 570) joy_dx = (raw_x - 570) / 450.0;
                            else if (raw_x < 450) joy_dx = (raw_x - 450) / 450.0;

                            if (raw_y > 570) joy_dy = (raw_y - 570) / 450.0;
                            else if (raw_y < 450) joy_dy = (raw_y - 450) / 450.0;

                            // Velocidad de desplazamiento: paso máximo en cm por ciclo
                            const double STEP = 0.25; 

                            // Actualizar posición de la pala de forma continua
                            current_x += joy_dx * STEP;
                            current_y += joy_dy * STEP;

                            // Límites seguros del espacio de trabajo
                            current_x = std::clamp(current_x, -4.5, 4.5);
                            current_y = std::clamp(current_y, 11.5, 16.8);
                        }

                        // Enviar comando al robot cada 2 lecturas para no saturar el bus
                        if (++loop_counter % 2 == 0) {
                            robot.moveToPlanar(current_x, current_y, GAMMA_PARALLEL, FIXED_SHOULDER);
                        }

                    } catch (...) {}
                }
                buffer = "";
            } else if (ch != '\r') {
                buffer += ch;
            }
        }
        usleep(5000); // 200 Hz polling serial
    }

    close(serial_fd);
    return 0;
}
