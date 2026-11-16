#include "pongbot/game_gui.hpp"

#include <cmath>
#include <cstdio>

GameGui::GameGui(int width, int height)
    : windowWidth(width), windowHeight(height),
      colorBackground(20, 20, 25),
      colorAccent(0, 255, 255),
      colorOk(0, 255, 120),
      colorError(50, 50, 255),
      colorText(220, 220, 220) {}

void GameGui::drawRetroText(cv::Mat& canvas, const std::string& text, cv::Point origin,
                            double fontScale, cv::Scalar color, int thickness) {
    // Sombra negra desplazada para contraste arcade
    cv::putText(canvas, text, origin + cv::Point(2, 2),
                cv::FONT_HERSHEY_DUPLEX, fontScale, cv::Scalar(0, 0, 0), thickness + 1, cv::LINE_AA);
    cv::putText(canvas, text, origin,
                cv::FONT_HERSHEY_DUPLEX, fontScale, color, thickness, cv::LINE_AA);
}

void GameGui::drawSystemCheckScreen(cv::Mat& canvas, const HardwareStatus& status, bool simulated) {
    canvas = colorBackground;

    drawRetroText(canvas, "PONGBOT SYSTEM HARDWARE CHECK", cv::Point(140, 70), 1.0, colorAccent, 2);
    if (simulated)
        drawRetroText(canvas, "MODO SIMULACION (sin hardware)", cv::Point(290, 100), 0.6, cv::Scalar(0, 200, 255), 1);
    cv::line(canvas, cv::Point(60, 115), cv::Point(900, 115), cv::Scalar(80, 80, 90), 2);

    int y = 160;
    for (const auto& d : status.devices) {
        drawRetroText(canvas, d.name, cv::Point(70, y), 0.62, colorText, 1);
        drawRetroText(canvas, "[" + d.device + "]", cv::Point(330, y), 0.55, cv::Scalar(150, 150, 150), 1);
        drawRetroText(canvas, d.ok ? "OK" : "NO", cv::Point(600, y), 0.7, d.ok ? colorOk : colorError, 2);
        if (!d.detail.empty()) {
            std::string detail = d.detail.size() > 70 ? d.detail.substr(0, 67) + "..." : d.detail;
            cv::putText(canvas, detail, cv::Point(90, y + 20), cv::FONT_HERSHEY_SIMPLEX, 0.42,
                        cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
        }
        y += 48;
        if (y > 420) break;
    }

    cv::line(canvas, cv::Point(60, 440), cv::Point(900, 440), cv::Scalar(80, 80, 90), 2);
    drawRetroText(canvas, "[R] Re-scan Hardware    |    [SPACE / ENTER] Continue to Menu",
                  cv::Point(170, 485), 0.55, cv::Scalar(180, 180, 180), 1);
}

void GameGui::drawMainMenuScreen(cv::Mat& canvas, int selectedMode,
                                 const std::vector<std::string>& modes,
                                 const HardwareStatus& status,
                                 const std::vector<MatchRecord>& recentMatches,
                                 const std::string& message) {
    canvas = colorBackground;

    drawRetroText(canvas, "=== PONGBOT ARCADE ARENA ===", cv::Point(210, 55), 1.1, colorAccent, 2);
    drawRetroText(canvas, "ECAM LaSalle - IT & Robotics Labs", cv::Point(330, 88), 0.55, cv::Scalar(170, 170, 170), 1);

    // Selector de modos (izquierda)
    cv::rectangle(canvas, cv::Rect(40, 110, 490, 300), cv::Scalar(35, 35, 45), -1);
    cv::rectangle(canvas, cv::Rect(40, 110, 490, 300), cv::Scalar(80, 80, 100), 2);
    drawRetroText(canvas, "SELECT GAME MODE:", cv::Point(60, 145), 0.7, colorAccent, 2);

    for (size_t i = 0; i < modes.size(); ++i) {
        int y = 190 + static_cast<int>(i) * 45;
        bool isSelected = (selectedMode == static_cast<int>(i + 1));
        cv::Scalar rowColor = isSelected ? colorOk : cv::Scalar(150, 150, 150);
        if (isSelected)
            cv::rectangle(canvas, cv::Rect(55, y - 28, 460, 38), cv::Scalar(45, 55, 45), -1);
        drawRetroText(canvas, (isSelected ? " > " : "   ") + std::to_string(i + 1) + ". " + modes[i],
                      cv::Point(62, y), 0.6, rowColor, isSelected ? 2 : 1);
    }

    // Historial (derecha)
    cv::rectangle(canvas, cv::Rect(550, 110, 370, 300), cv::Scalar(35, 35, 45), -1);
    cv::rectangle(canvas, cv::Rect(550, 110, 370, 300), cv::Scalar(80, 80, 100), 2);
    drawRetroText(canvas, "RECENT MATCHES", cv::Point(570, 145), 0.7, colorAccent, 2);
    int histY = 185;
    if (recentMatches.empty()) {
        drawRetroText(canvas, "No match history recorded", cv::Point(570, histY), 0.55, cv::Scalar(130, 130, 130), 1);
    } else {
        for (const auto& match : recentMatches) {
            std::string line = match.winner + " (" + std::to_string(match.scoreP1) + "-" +
                               std::to_string(match.scoreP2) + ")";
            drawRetroText(canvas, line, cv::Point(570, histY), 0.5, colorText, 1);
            cv::putText(canvas, match.mode, cv::Point(585, histY + 17), cv::FONT_HERSHEY_SIMPLEX, 0.4,
                        cv::Scalar(140, 140, 140), 1, cv::LINE_AA);
            histY += 44;
        }
    }

    if (!message.empty())
        drawRetroText(canvas, message, cv::Point(50, 440), 0.55, cv::Scalar(0, 180, 255), 1);

    cv::line(canvas, cv::Point(40, 455), cv::Point(920, 455), cv::Scalar(70, 70, 80), 2);
    auto drawBadge = [&](const std::string& label, bool ok, int x) {
        drawRetroText(canvas, label + ": " + (ok ? "OK" : "NO"), cv::Point(x, 490), 0.52, ok ? colorOk : colorError, 1);
    };
    drawBadge("CAM", status.cameraOk, 50);
    drawBadge("DXL", status.dynamixelOk, 170);
    drawBadge("JOY", status.joystickOk, 290);
    drawRetroText(canvas, "[UP/DOWN/1-5] Mode | [ENTER] Start | [R] Check | [ESC] Exit",
                  cv::Point(410, 490), 0.45, cv::Scalar(180, 180, 180), 1);
}

void GameGui::drawMatchHud(cv::Mat& canvas, int scoreP1, int scoreP2,
                           int matchTimeSec, const std::string& activeMode,
                           const std::string& p1Name, const std::string& p2Name) {
    cv::rectangle(canvas, cv::Rect(0, 0, windowWidth, 70), cv::Scalar(25, 25, 30), -1);
    cv::line(canvas, cv::Point(0, 70), cv::Point(windowWidth, 70), cv::Scalar(80, 80, 90), 2);

    drawRetroText(canvas, p1Name + ": " + std::to_string(scoreP1), cv::Point(30, 50), 0.9, colorOk, 2);
    char timeBuffer[16];
    std::snprintf(timeBuffer, sizeof(timeBuffer), "%02d:%02d", matchTimeSec / 60, matchTimeSec % 60);
    drawRetroText(canvas, std::string(timeBuffer), cv::Point(440, 50), 0.9, colorAccent, 2);
    drawRetroText(canvas, p2Name + ": " + std::to_string(scoreP2), cv::Point(680, 50), 0.9,
                  cv::Scalar(255, 100, 100), 2);
    drawRetroText(canvas, activeMode, cv::Point(330, 20), 0.45, cv::Scalar(180, 180, 180), 1);

    cv::rectangle(canvas, cv::Rect(0, windowHeight - 26, windowWidth, 26), cv::Scalar(25, 25, 30), -1);
    cv::putText(canvas,
                "[SPACE] Gol P1  [B] Gol P2  [X] Armar/Desarmar  [H] Home  [C] Recalibrar ArUco  [M] Menu  [ESC] Salir",
                cv::Point(12, windowHeight - 8), cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(200, 200, 200), 1,
                cv::LINE_AA);
}

void GameGui::drawPlan(cv::Mat& canvas, const cv::Rect& area, const PlanView& p) {
    cv::Mat roi = canvas(area);
    roi = cv::Scalar(40, 45, 40);
    const double L = p.half_length + 1.0, W = p.half_width + 2.0;
    const double sx = area.width / (2 * L), sy = area.height / (2 * W);
    auto P = [&](double X, double Y) {
        return cv::Point(static_cast<int>(area.x + (X + L) * sx),
                         static_cast<int>(area.y + (W - Y) * sy));  // +Y hacia arriba
    };
    cv::rectangle(canvas, P(-p.half_length, p.half_width), P(p.half_length, -p.half_width),
                  cv::Scalar(90, 120, 90), 1);
    cv::line(canvas, P(0, p.half_width), P(0, -p.half_width), cv::Scalar(70, 90, 70), 1);
    for (const auto& m : p.markers)
        cv::drawMarker(canvas, P(m.x, m.y), cv::Scalar(150, 150, 150), cv::MARKER_SQUARE, 8, 1);

    for (const auto& a : p.arms) {
        // línea de gol
        const double gx = a.side * a.goal_u;
        cv::line(canvas, P(gx, p.half_width), P(gx, -p.half_width), cv::Scalar(60, 60, 200), 1);
        // pala
        const double px = a.side * a.paddle_u;
        cv::Scalar col = !a.armed ? cv::Scalar(120, 120, 120)
                                  : (a.smashing ? cv::Scalar(0, 140, 255) : cv::Scalar(255, 200, 0));
        cv::line(canvas, P(px, a.paddle_y - 2.5), P(px, a.paddle_y + 2.5), col, 3);
        cv::putText(canvas, a.label, P(px + a.side * 1.5, a.paddle_y + 3.5), cv::FONT_HERSHEY_SIMPLEX,
                    0.4, col, 1, cv::LINE_AA);
    }
    if (p.ball_valid) {
        cv::circle(canvas, P(p.ball.x, p.ball.y), 5, cv::Scalar(0, 120, 255), -1);
        if (p.vel_valid)
            cv::arrowedLine(canvas, P(p.ball.x, p.ball.y),
                            P(p.ball.x + p.vel.x * 0.25, p.ball.y + p.vel.y * 0.25),
                            cv::Scalar(0, 200, 255), 1);
        char buf[64];
        std::snprintf(buf, sizeof(buf), "ball X=%.1f Y=%.1f", p.ball.x, p.ball.y);
        cv::putText(canvas, buf, cv::Point(area.x + 5, area.y + area.height - 6), cv::FONT_HERSHEY_SIMPLEX,
                    0.4, cv::Scalar(220, 220, 220), 1, cv::LINE_AA);
    }
}

void GameGui::drawGameOverScreen(cv::Mat& canvas, const std::string& winner,
                                 int scoreP1, int scoreP2) {
    canvas = colorBackground;
    drawRetroText(canvas, "=== MATCH COMPLETED ===", cv::Point(240, 120), 1.1, colorAccent, 2);
    drawRetroText(canvas, "WINNER: " + winner, cv::Point(290, 220), 0.9, colorOk, 2);
    drawRetroText(canvas, "Final Score: " + std::to_string(scoreP1) + " - " + std::to_string(scoreP2),
                  cv::Point(340, 280), 0.75, colorText, 1);
    drawRetroText(canvas, "Press [ENTER] or [SPACE] to return to Main Menu",
                  cv::Point(220, 420), 0.6, cv::Scalar(180, 180, 180), 1);
}
