#include "pong_robot.hpp"
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <chrono>

#define ARDUINO_PORT "/dev/ttyACM0"

// Configure serial port for non-blocking raw reading at 115200 baud
int openSerial(const char* portName) {
    int fd = open(portName, O_RDWR | O_NOCTTY | O_NONBLOCK);
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
    tcflush(fd, TCIFLUSH);
    return fd;
}

// Discard buffered stale packets and extract only the latest reading
bool getLatestJoystickData(int fd, int &raw_x, int &raw_y) {
    char buf[512];
    int n = read(fd, buf, sizeof(buf) - 1);
    if (n <= 0) return false;

    buf[n] = '\0';
    std::string data(buf);

    size_t last_nl = data.rfind('\n');
    if (last_nl == std::string::npos) return false;

    size_t prev_nl = data.rfind('\n', last_nl - 1);
    std::string last_line;
    if (prev_nl != std::string::npos) {
        last_line = data.substr(prev_nl + 1, last_nl - prev_nl - 1);
    } else {
        last_line = data.substr(0, last_nl);
    }

    std::stringstream ss(last_line);
    std::string s_x, s_y;
    if (std::getline(ss, s_x, ',') && std::getline(ss, s_y, ',')) {
        try {
            raw_x = std::stoi(s_x);
            raw_y = std::stoi(s_y);
            return true;
        } catch (...) {
            return false;
        }
    }
    return false;
}

int main() {
    // Measured link lengths in cm: L1 = 5.0, L2 = 5.5, L3 = 6.3
    PongRobot robot(5.0, 5.5, 6.3, 1, 2, 3, 4);
    if (!robot.init()) {
        std::cerr << "[-] Error initializing Dynamixel communication." << std::endl;
        return -1;
    }

    int serial_fd = openSerial(ARDUINO_PORT);
    if (serial_fd == -1) {
        std::cerr << "[-] Failed to open " << ARDUINO_PORT << ". Check USB connection/permissions." << std::endl;
        return -1;
    }

    const int FIXED_SHOULDER = 341;
    const double GAMMA_PARALLEL = 0.0; // Paddle stays parallel to the baseline (x0)

    // Base defense position
    const double DEFAULT_Y = 13.5;
    double current_x = 0.0;
    double current_y = DEFAULT_Y;

    std::cout << "[+] Positioning robot to initial home state (X = 0.0, Y = " << DEFAULT_Y << " cm)..." << std::endl;
    robot.moveToPlanar(current_x, current_y, GAMMA_PARALLEL, FIXED_SHOULDER);
    sleep(1);

    std::cout << "\n=======================================================" << std::endl;
    std::cout << "  HIGH-SPEED 1D DEFENSE + SMASH TRIGGER CONTROLLER     " << std::endl;
    std::cout << "  - Move joystick LEFT / RIGHT : Rapid X translation   " << std::endl;
    std::cout << "  - Push joystick FORWARD (Y)  : Trigger SMASH attack  " << std::endl;
    std::cout << "  - Exit                       : Ctrl + C              " << std::endl;
    std::cout << "=======================================================" << std::endl;

    int raw_x = 512, raw_y = 512;
    bool smash_active = false;
    auto last_motor_update = std::chrono::steady_clock::now();

    while (true) {
        if (getLatestJoystickData(serial_fd, raw_x, raw_y)) {

            // SMASH ATTACK TRIGGER: Push joystick forward (raw_y > 800)
            if (raw_y > 800 && !smash_active) {
                smash_active = true;
                std::cout << ">>> [SMASH] Executing rapid forward strike at X = " << current_x << " cm" << std::endl;

                // 1. Rapid backward recoil (-Y) to build momentum
                robot.moveToPlanar(current_x, 11.5, GAMMA_PARALLEL, FIXED_SHOULDER);
                usleep(35000); // 35 ms

                // 2. High-speed forward impact strike (+Y)
                robot.moveToPlanar(current_x, 15.3, GAMMA_PARALLEL, FIXED_SHOULDER);
                usleep(55000); // 55 ms

                // 3. Return to nominal defensive baseline
                current_y = DEFAULT_Y;
                robot.moveToPlanar(current_x, current_y, GAMMA_PARALLEL, FIXED_SHOULDER);

                // Flush serial buffer to prevent repeated triggers
                tcflush(serial_fd, TCIFLUSH);
                smash_active = false;
            }
            else if (!smash_active) {
                // LATERAL X MOVEMENT ONLY
                double vx = 0.0;

                // Deadzone around joystick resting position (450 to 570)
                if (raw_x > 570) vx = (raw_x - 570.0) / 450.0;
                else if (raw_x < 450) vx = (raw_x - 450.0) / 450.0;

                // High speed displacement step
                const double SPEED = 0.85; // Step in cm per update cycle
                double next_x = current_x + (vx * SPEED);

                // Safe defensive boundary limits
                next_x = std::clamp(next_x, -4.2, 4.2);

                double q1, q2, q3;
                if (robot.computeIK(next_x, DEFAULT_Y, GAMMA_PARALLEL, q1, q2, q3)) {
                    current_x = next_x;
                    current_y = DEFAULT_Y;
                }
            }
        }

        // Send motor position updates every 15 ms (~66 Hz refresh rate)
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_motor_update).count();

        if (elapsed >= 15 && !smash_active) {
            robot.moveToPlanar(current_x, current_y, GAMMA_PARALLEL, FIXED_SHOULDER);
            last_motor_update = now;
        }

        usleep(2000); // 2 ms polling loop
    }

    close(serial_fd);
    return 0;
}
