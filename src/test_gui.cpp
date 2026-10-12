#include "game_gui.hpp"
#include "score_manager.hpp"
#include <iostream>

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
    int matchSeconds = 0;
    std::string winnerName = "";

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
            // Fondo de juego simulado
            canvas = cv::Scalar(25, 25, 30);
            cv::rectangle(canvas, cv::Rect(100, 100, 760, 380), cv::Scalar(40, 40, 50), -1);
            cv::rectangle(canvas, cv::Rect(100, 100, 760, 380), cv::Scalar(80, 80, 90), 2);
            gui.drawRetroText(canvas, "[Active Match Area / Camera Feed Layer]", 
                              cv::Point(240, 290), 0.7, cv::Scalar(120, 120, 120), 1);

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
            } else if (key == 13 || key == 32) { // ENTER o SPACE -> ir a Menú
                currentState = GuiState::MAIN_MENU;
            }
        } 
        else if (currentState == GuiState::MAIN_MENU) {
            if (key == 82 || key == 'w' || key == 'W') { // Arriba
                selectedMode = (selectedMode > 1) ? selectedMode - 1 : 3;
            } else if (key == 84 || key == 's' || key == 'S') { // Abajo
                selectedMode = (selectedMode < 3) ? selectedMode + 1 : 1;
            } else if (key == 13 || key == 32) { // ENTER o SPACE -> Jugar
                scoreP1 = 0;
                scoreP2 = 0;
                matchSeconds = 15; // Simulado
                currentState = GuiState::PLAYING_MATCH;
            }
        } 
        else if (currentState == GuiState::PLAYING_MATCH) {
            if (key == 'm' || key == 'M') {
                currentState = GuiState::MAIN_MENU;
            } else if (key == 32) { // SPACE -> Gol P1
                scoreP1++;
            } else if (key == 'b' || key == 'B') { // B -> Gol P2
                scoreP2++;
            }

            // Regla de victoria (primero en llegar a 5 puntos)
            if (scoreP1 >= 5 || scoreP2 >= 5) {
                winnerName = (scoreP1 >= 5) ? "Player 1" : "Player 2 / Robot";
                std::string modeLabel = (selectedMode == 1) ? "Mode 1: Manual" :
                                        (selectedMode == 2) ? "Mode 2: Semi-Auto" : "Mode 3: Autonomous";
                scoreMgr.saveMatch(modeLabel, scoreP1, scoreP2, winnerName, matchSeconds);
                currentState = GuiState::GAME_OVER;
            }
        } 
        else if (currentState == GuiState::GAME_OVER) {
            if (key == 13 || key == 32) { // ENTER o SPACE -> Volver al menú
                currentState = GuiState::MAIN_MENU;
            }
        }
    }

    cv::destroyAllWindows();
    return 0;
}
