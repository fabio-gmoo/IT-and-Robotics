#include "game_gui.hpp"
#include "score_manager.hpp"
#include <iostream>
#include <chrono>

int main() {
    std::cout << "[GUI] Launching Arcade Interface Prototype..." << std::endl;

    const int WIDTH = 960;
    const int HEIGHT = 540;
    cv::Mat canvas(HEIGHT, WIDTH, CV_8UC3);

    GameGui gui(WIDTH, HEIGHT);
    ScoreManager scoreMgr("data/scores.csv");

    GuiState currentState = GuiState::SYSTEM_CHECK;
    HardwareStatus hwStatus = gui.scanHardware();
    int selectedMode = 1;
    int scoreP1 = 0;
    int scoreP2 = 0;
    std::string winnerName = "";

    // Timer para la partida
    std::chrono::steady_clock::time_point matchStartTime;

    // Intento de captura de video para la capa en vivo
    cv::VideoCapture cap;
    bool cameraActive = false;

    cv::namedWindow("PongBot Arcade GUI", cv::WINDOW_AUTOSIZE);

    while (true) {
        if (currentState == GuiState::SYSTEM_CHECK) {
            gui.drawSystemCheckScreen(canvas, hwStatus);
        } 
        else if (currentState == GuiState::MAIN_MENU) {
            std::vector<MatchRecord> recent = scoreMgr.loadRecentMatches(5);
            gui.drawMainMenuScreen(canvas, selectedMode, hwStatus, recent);
        } 
        else if (currentState == GuiState::PLAYING_MATCH) {
            // Calcular duracion transcurrida en segundos
            auto now = std::chrono::steady_clock::now();
            int matchSeconds = std::chrono::duration_cast<std::chrono::seconds>(now - matchStartTime).count();

            // Capa de fondo: Video en vivo o simulacion
            if (cameraActive && cap.isOpened()) {
                cv::Mat frame;
                if (cap.read(frame) && !frame.empty()) {
                    cv::resize(frame, canvas, cv::Size(WIDTH, HEIGHT));
                } else {
                    canvas = cv::Scalar(25, 25, 30);
                }
            } else {
                canvas = cv::Scalar(25, 25, 30);
                cv::rectangle(canvas, cv::Rect(100, 100, 760, 380), cv::Scalar(40, 40, 50), -1);
                cv::rectangle(canvas, cv::Rect(100, 100, 760, 380), cv::Scalar(80, 80, 90), 2);
                gui.drawRetroText(canvas, "[Simulation Mode: Camera Offline]", 
                                  cv::Point(280, 290), 0.7, cv::Scalar(120, 120, 120), 1);
            }

            std::string modeLabel = (selectedMode == 1) ? "MODE 1: DUAL MANUAL" :
                                    (selectedMode == 2) ? "MODE 2: SEMI-AUTO" : "MODE 3: AUTONOMOUS";
            gui.drawMatchHud(canvas, scoreP1, scoreP2, matchSeconds, modeLabel);
        } 
        else if (currentState == GuiState::GAME_OVER) {
            gui.drawGameOverScreen(canvas, winnerName, scoreP1, scoreP2);
        }

        cv::imshow("PongBot Arcade GUI", canvas);
        int key = cv::waitKey(30);

        if (key == 27) { // ESC -> Salir
            break;
        }

        if (currentState == GuiState::SYSTEM_CHECK) {
            if (key == 'r' || key == 'R') {
                hwStatus = gui.scanHardware();
                std::cout << "[GUI] Hardware re-scanned." << std::endl;
            } else if (key == 13 || key == 32) { // ENTER o SPACE -> ir a Menu
                currentState = GuiState::MAIN_MENU;
            }
        } 
        else if (currentState == GuiState::MAIN_MENU) {
            if (key == 82 || key == 'w' || key == 'W') { // Arriba
                selectedMode = (selectedMode > 1) ? selectedMode - 1 : 3;
            } else if (key == 84 || key == 's' || key == 'S') { // Abajo
                selectedMode = (selectedMode < 3) ? selectedMode + 1 : 1;
            } else if (key == 13 || key == 32) { // ENTER o SPACE -> Iniciar Match
                scoreP1 = 0;
                scoreP2 = 0;
                matchStartTime = std::chrono::steady_clock::now();

                // Intentar abrir la camara si esta presente
                if (hwStatus.cameraOk && !cap.isOpened()) {
                    cameraActive = cap.open(0, cv::CAP_V4L2);
                }

                currentState = GuiState::PLAYING_MATCH;
            }
        } 
        else if (currentState == GuiState::PLAYING_MATCH) {
            if (key == 'm' || key == 'M') {
                if (cap.isOpened()) cap.release();
                cameraActive = false;
                currentState = GuiState::MAIN_MENU;
            } else if (key == 32) { // SPACE -> Gol P1
                scoreP1++;
            } else if (key == 'b' || key == 'B') { // B -> Gol P2
                scoreP2++;
            }

            // Regla de fin de partida (5 goles)
            if (scoreP1 >= 5 || scoreP2 >= 5) {
                auto now = std::chrono::steady_clock::now();
                int totalSec = std::chrono::duration_cast<std::chrono::seconds>(now - matchStartTime).count();

                winnerName = (scoreP1 >= 5) ? "Player 1" : "Player 2 / Robot";
                std::string modeLabel = (selectedMode == 1) ? "Mode 1: Manual" :
                                        (selectedMode == 2) ? "Mode 2: Semi-Auto" : "Mode 3: Autonomous";
                scoreMgr.saveMatch(modeLabel, scoreP1, scoreP2, winnerName, totalSec);

                if (cap.isOpened()) cap.release();
                cameraActive = false;
                currentState = GuiState::GAME_OVER;
            }
        } 
        else if (currentState == GuiState::GAME_OVER) {
            if (key == 13 || key == 32) { // ENTER o SPACE -> Volver al menu
                currentState = GuiState::MAIN_MENU;
            }
        }
    }

    if (cap.isOpened()) cap.release();
    cv::destroyAllWindows();
    return 0;
}
