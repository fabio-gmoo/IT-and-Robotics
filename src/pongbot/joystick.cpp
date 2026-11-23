#include "pongbot/joystick.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <sstream>

namespace pongbot {

std::map<std::string, std::weak_ptr<SerialLineReader>>& SerialLineReader::registry() {
    static std::map<std::string, std::weak_ptr<SerialLineReader>> r;
    return r;
}

std::shared_ptr<SerialLineReader> SerialLineReader::open(const std::string& device,
                                                         std::string& error) {
    auto& reg = registry();
    auto it = reg.find(device);
    if (it != reg.end())
        if (auto existing = it->second.lock()) return existing;

    // Misma configuración que joystick_teleop.cpp (raw, 8N1, 115200, no bloqueante)
    int fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd < 0) {
        error = "Cannot open " + device + " (Arduino plugged in? 'dialout' permissions?)";
        return nullptr;
    }
    termios options{};
    tcgetattr(fd, &options);
    cfsetispeed(&options, B115200);
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    options.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    options.c_oflag &= ~OPOST;
    tcsetattr(fd, TCSANOW, &options);
    tcflush(fd, TCIFLUSH);

    std::shared_ptr<SerialLineReader> r(new SerialLineReader());
    r->fd_ = fd;
    r->device_ = device;
    reg[device] = r;
    return r;
}

SerialLineReader::~SerialLineReader() {
    if (fd_ >= 0) ::close(fd_);
}

bool SerialLineReader::writeLine(const std::string& line) {
    if (fd_ < 0) return false;
    const std::string out = line + "\n";
    return ::write(fd_, out.data(), out.size()) == static_cast<ssize_t>(out.size());
}

void SerialLineReader::poll() {
    if (fd_ < 0) return;
    char buf[512];
    while (true) {
        const ssize_t n = ::read(fd_, buf, sizeof(buf));
        if (n <= 0) break;
        buffer_.append(buf, static_cast<size_t>(n));
    }
    if (buffer_.size() > 4096) buffer_.erase(0, buffer_.size() - 1024);

    // Busca la última línea COMPLETA (descarta las viejas => sin retraso).
    const size_t last_nl = buffer_.rfind('\n');
    if (last_nl == std::string::npos) return;
    const size_t prev_nl = (last_nl == 0) ? std::string::npos : buffer_.rfind('\n', last_nl - 1);
    const size_t start = (prev_nl == std::string::npos) ? 0 : prev_nl + 1;
    std::string line = buffer_.substr(start, last_nl - start);
    buffer_.erase(0, last_nl + 1);

    std::vector<int> vals;
    std::stringstream ss(line);
    std::string item;
    while (std::getline(ss, item, ',')) {
        try {
            vals.push_back(std::stoi(item));
        } catch (...) {
            return;  // línea corrupta: mantenemos los valores anteriores
        }
    }
    if (vals.size() >= 2) values_ = vals;
}

}  // namespace pongbot
