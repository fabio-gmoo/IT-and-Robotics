#ifndef GAME_GUI_HPP
#define GAME_GUI_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "pongbot/score_manager.hpp"

enum class GuiState {
    SYSTEM_CHECK,
    MAIN_MENU,
    PLAYING_MATCH,
    GAME_OVER
};

// Una fila de la pantalla de comprobación de hardware
struct DeviceStatus {
    std::string name;     // "Brazo A", "Joystick P1", ...
    std::string device;   // "/dev/ttyUSB0"
    bool ok = false;
    std::string detail;   // mensaje de error / info
};

struct HardwareStatus {
    std::vector<DeviceStatus> devices;
    bool cameraOk = false;
    bool dynamixelOk = false;
    bool joystickOk = false;
};

// Datos para dibujar la vista cenital de la mesa
struct PlanArm {
    std::string label;
    int side = 1;
    double paddle_u = 0.0;     // u actual de la pala
    double paddle_y = 0.0;     // Y de mesa de la pala
    double goal_u = 23.5;
    bool smashing = false;
    bool armed = true;
};

struct PlanView {
    double half_length = 24.0;
    double half_width = 8.15;
    std::vector<cv::Point2f> markers;
    bool ball_valid = false;
    cv::Point2f ball;
    cv::Point2f vel;
    bool vel_valid = false;
    std::vector<PlanArm> arms;
};

class GameGui {
private:
    int windowWidth;
    int windowHeight;
    cv::Scalar colorBackground;
    cv::Scalar colorAccent;
    cv::Scalar colorOk;
    cv::Scalar colorError;
    cv::Scalar colorText;

public:
    GameGui(int width = 960, int height = 540);

    void drawRetroText(cv::Mat& canvas, const std::string& text, cv::Point origin,
                       double fontScale, cv::Scalar color, int thickness = 2);

    void drawSystemCheckScreen(cv::Mat& canvas, const HardwareStatus& status, bool simulated);

    void drawMainMenuScreen(cv::Mat& canvas, int selectedMode,
                            const std::vector<std::string>& modes,
                            const HardwareStatus& status,
                            const std::vector<MatchRecord>& recentMatches,
                            const std::string& message);

    void drawMatchHud(cv::Mat& canvas, int scoreP1, int scoreP2,
                      int matchTimeSec, const std::string& activeMode,
                      const std::string& p1Name, const std::string& p2Name);

    void drawPlan(cv::Mat& canvas, const cv::Rect& area, const PlanView& plan);

    void drawGameOverScreen(cv::Mat& canvas, const std::string& winner,
                            int scoreP1, int scoreP2);
};

#endif // GAME_GUI_HPP
