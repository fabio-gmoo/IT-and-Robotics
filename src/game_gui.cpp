#include "game_gui.hpp"
#include <unistd.h>
#include <fcntl.h>

GameGui::GameGui(int width, int height)
    : windowWidth(width), windowHeight(height),
      colorBackground(20, 20, 25),
      colorAccent(0, 255, 255),
      colorOk(0, 255, 120),
      colorError(50, 50, 255),
      colorText(220, 220, 220) {}

bool GameGui::checkDeviceNode(const std::string& devicePath) {
    return (access(devicePath.c_str(), F_OK) == 0);
}

HardwareStatus GameGui::scanHardware() {
    HardwareStatus status;
    status.cameraOk = checkDeviceNode("/dev/video0");
    status.dynamixelOk = checkDeviceNode("/dev/ttyUSB0");
    status.joystickOk = checkDeviceNode("/dev/ttyACM0");
    return status;
}

void GameGui::drawRetroText(cv::Mat& canvas, const std::string& text, cv::Point origin, 
                           double fontScale, cv::Scalar color, int thickness) {
    // Sombra negra desplazada para contraste arcade
    cv::putText(canvas, text, origin + cv::Point(2, 2), 
                cv::FONT_HERSHEY_DUPLEX, fontScale, cv::Scalar(0, 0, 0), thickness + 1, cv::LINE_AA);
    cv::putText(canvas, text, origin, 
                cv::FONT_HERSHEY_DUPLEX, fontScale, color, thickness, cv::LINE_AA);
}

void GameGui::drawSystemCheckScreen(cv::Mat& canvas, const HardwareStatus& status) {
    canvas = colorBackground;

    // Encabezado
    drawRetroText(canvas, "PONGBOT SYSTEM HARDWARE CHECK", cv::Point(140, 80), 1.0, colorAccent, 2);
    cv::line(canvas, cv::Point(80, 110), cv::Point(880, 110), cv::Scalar(80, 80, 90), 2);

    // Lista de diagnósticos
    int yOffset = 180;
    auto drawStatusRow = [&](const std::string& name, const std::string& port, bool ok, int y) {
        drawRetroText(canvas, name, cv::Point(100, y), 0.7, colorText, 1);
        drawRetroText(canvas, "[" + port + "]", cv::Point(430, y), 0.65, cv::Scalar(150, 150, 150), 1);
        cv::Scalar statusColor = ok ? colorOk : colorError;
        std::string statusMsg = ok ? "CONNECTED" : "DISCONNECTED";
        drawRetroText(canvas, statusMsg, cv::Point(680, y), 0.7, statusColor, 2);
    };

    drawStatusRow("Overhead Camera", "/dev/video0", status.cameraOk, yOffset);
    drawStatusRow("Dynamixel U2D2 Bus", "/dev/ttyUSB0", status.dynamixelOk, yOffset + 60);
    drawStatusRow("Arduino Joystick", "/dev/ttyACM0", status.joystickOk, yOffset + 120);

    // Instrucciones inferiores
    cv::line(canvas, cv::Point(80, 430), cv::Point(880, 430), cv::Scalar(80, 80, 90), 2);
    drawRetroText(canvas, "[R] Re-scan Hardware    |    [SPACE / ENTER] Continue to Menu", 
                  cv::Point(170, 475), 0.55, cv::Scalar(180, 180, 180), 1);
}

void GameGui::drawMainMenuScreen(cv::Mat& canvas, int selectedMode, 
                                 const HardwareStatus& status, 
                                 const std::vector<MatchRecord>& recentMatches) {
    canvas = colorBackground;

    // Banner principal
    drawRetroText(canvas, "=== PONGBOT ARCADE ARENA ===", cv::Point(210, 60), 1.1, colorAccent, 2);
    drawRetroText(canvas, "ECAM LaSalle - IT & Robotics Labs", cv::Point(330, 95), 0.55, cv::Scalar(170, 170, 170), 1);

    // Selector de Modos (Lado Izquierdo)
    cv::rectangle(canvas, cv::Rect(50, 130, 470, 260), cv::Scalar(35, 35, 45), -1);
    cv::rectangle(canvas, cv::Rect(50, 130, 470, 260), cv::Scalar(80, 80, 100), 2);
    drawRetroText(canvas, "SELECT GAME MODE:", cv::Point(70, 165), 0.7, colorAccent, 2);

    std::vector<std::string> modes = {
        "1. Mode 1: Dual Manual (2P)",
        "2. Mode 2: Semi-Autonomous (Human vs AI)",
        "3. Mode 3: Fully Autonomous (AI vs AI)"
    };

    for (size_t i = 0; i < modes.size(); ++i) {
        int y = 220 + static_cast<int>(i) * 55;
        bool isSelected = (selectedMode == static_cast<int>(i + 1));
        cv::Scalar rowColor = isSelected ? colorOk : cv::Scalar(150, 150, 150);
        std::string prefix = isSelected ? " > " : "   ";
        
        if (isSelected) {
            cv::rectangle(canvas, cv::Rect(65, y - 28, 440, 40), cv::Scalar(45, 55, 45), -1);
        }
        drawRetroText(canvas, prefix + modes[i], cv::Point(75, y), 0.65, rowColor, isSelected ? 2 : 1);
    }

    // Panel de Historial Reciente (Lado Derecho)
    cv::rectangle(canvas, cv::Rect(540, 130, 370, 260), cv::Scalar(35, 35, 45), -1);
    cv::rectangle(canvas, cv::Rect(540, 130, 370, 260), cv::Scalar(80, 80, 100), 2);
    drawRetroText(canvas, "RECENT MATCHES", cv::Point(560, 165), 0.7, colorAccent, 2);

    int histY = 205;
    if (recentMatches.empty()) {
        drawRetroText(canvas, "No match history recorded", cv::Point(560, histY), 0.55, cv::Scalar(130, 130, 130), 1);
    } else {
        for (const auto& match : recentMatches) {
            std::string line = match.winner + " won (" + std::to_string(match.scoreP1) + "-" + std::to_string(match.scoreP2) + ")";
            drawRetroText(canvas, line, cv::Point(560, histY), 0.52, colorText, 1);
            histY += 35;
        }
    }

    // Barra de Estado y Navegación
    cv::line(canvas, cv::Point(50, 420), cv::Point(910, 420), cv::Scalar(70, 70, 80), 2);

    auto drawBadge = [&](const std::string& label, bool ok, int x) {
        cv::Scalar badgeColor = ok ? colorOk : colorError;
        std::string text = label + ": " + (ok ? "OK" : "NO");
        drawRetroText(canvas, text, cv::Point(x, 455), 0.52, badgeColor, 1);
    };

    drawBadge("CAM", status.cameraOk, 60);
    drawBadge("U2D2", status.dynamixelOk, 190);
    drawBadge("JOY", status.joystickOk, 330);

    drawRetroText(canvas, "[UP/DOWN] Mode  |  [ENTER] Start Match  |  [ESC] Exit", 
                  cv::Point(440, 455), 0.5, cv::Scalar(180, 180, 180), 1);
}

void GameGui::drawMatchHud(cv::Mat& canvas, int scoreP1, int scoreP2, 
                           int matchTimeSec, const std::string& activeMode) {
    // Barra superior del marcador
    cv::rectangle(canvas, cv::Rect(0, 0, windowWidth, 75), cv::Scalar(25, 25, 30), -1);
    cv::line(canvas, cv::Point(0, 75), cv::Point(windowWidth, 75), cv::Scalar(80, 80, 90), 2);

    // Marcador
    drawRetroText(canvas, "P1: " + std::to_string(scoreP1), cv::Point(60, 48), 1.0, colorOk, 2);
    
    int min = matchTimeSec / 60;
    int sec = matchTimeSec % 60;
    char timeBuffer[16];
    std::snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d", min, sec);
    drawRetroText(canvas, std::string(timeBuffer), cv::Point(435, 48), 0.9, colorAccent, 2);

    drawRetroText(canvas, "P2: " + std::to_string(scoreP2), cv::Point(800, 48), 1.0, cv::Scalar(255, 100, 100), 2);

    // Modo activo
    drawRetroText(canvas, activeMode, cv::Point(330, 25), 0.5, cv::Scalar(180, 180, 180), 1);

    // Pie de control
    drawRetroText(canvas, "[SPACE] Goal P1  |  [B] Goal P2  |  [M] Back to Menu", 
                  cv::Point(230, windowHeight - 20), 0.5, cv::Scalar(200, 200, 200), 1);
}

void GameGui::drawGameOverScreen(cv::Mat& canvas, const std::string& winner, 
                                 int scoreP1, int scoreP2) {
    canvas = colorBackground;
    drawRetroText(canvas, "=== MATCH COMPLETED ===", cv::Point(240, 120), 1.1, colorAccent, 2);

    std::string winnerText = "WINNER: " + winner;
    drawRetroText(canvas, winnerText, cv::Point(310, 220), 0.9, colorOk, 2);

    std::string scoreSummary = "Final Score: " + std::to_string(scoreP1) + " - " + std::to_string(scoreP2);
    drawRetroText(canvas, scoreSummary, cv::Point(340, 280), 0.75, colorText, 1);

    drawRetroText(canvas, "Press [ENTER] or [SPACE] to return to Main Menu", 
                  cv::Point(220, 420), 0.6, cv::Scalar(180, 180, 180), 1);
}
