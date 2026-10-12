#ifndef GAME_GUI_HPP
#define GAME_GUI_HPP

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include "score_manager.hpp"

enum class GuiState {
    SYSTEM_CHECK,
    MAIN_MENU,
    PLAYING_MATCH,
    GAME_OVER
};

struct HardwareStatus {
    bool cameraOk = false;
    bool dynamixelOk = false;
    bool joystickOk = false;
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

    bool checkDeviceNode(const std::string& devicePath);

public:
    GameGui(int width = 960, int height = 540);

    HardwareStatus scanHardware();

    void drawRetroText(cv::Mat& canvas, const std::string& text, cv::Point origin, 
                       double fontScale, cv::Scalar color, int thickness = 2);

    void drawSystemCheckScreen(cv::Mat& canvas, const HardwareStatus& status);

    void drawMainMenuScreen(cv::Mat& canvas, int selectedMode, 
                            const HardwareStatus& status, 
                            const std::vector<MatchRecord>& recentMatches);

    void drawMatchHud(cv::Mat& canvas, int scoreP1, int scoreP2, 
                      int matchTimeSec, const std::string& activeMode);

    void drawGameOverScreen(cv::Mat& canvas, const std::string& winner, 
                            int scoreP1, int scoreP2);
};

#endif // GAME_GUI_HPP
