#include "pong_robot.hpp"
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <chrono>
#include <cmath>

// Configuration Constants
constexpr float ROBOT_X = 24.0f; // Distancia corregida: 24 cm desde el centro de la cancha
constexpr float ROBOT_Y = 0.0f;
constexpr int MIN_BALL_PIXELS = 150;
constexpr double FPS = 30.0;

const std::string COLOR_FILE = "./data/color_params_data.xml";
const std::string CAMERA_FILE = "./data/camera_calibration_params.xml";

// ArUco Markers world coordinates for table mapping
const std::vector<cv::Point2f> WORLD_MARKERS = {
    {-20.4f,  8.15f}, {20.4f,  8.15f},
    {-20.4f, -8.15f}, {20.4f, -8.15f}
};

bool readColorParameters(int &loH, int &hiH, int &loS, int &hiS, int &loV, int &hiV) {
    cv::FileStorage fs(COLOR_FILE, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["lowH"] >> loH; fs["highH"] >> hiH;
    fs["lowS"] >> loS; fs["highS"] >> hiS;
    fs["lowV"] >> loV; fs["highV"] >> hiV;
    return true;
}

bool readCameraParameters(cv::Mat &matrix, cv::Mat &dist) {
    cv::FileStorage fs(CAMERA_FILE, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["camera_matrix"] >> matrix;
    fs["distortion_coefficients"] >> dist;
    return !matrix.empty() && !dist.empty();
}

bool sortedArUcoCenters(const std::vector<std::vector<cv::Point2f>> &corners, std::vector<cv::Point2f> &sorted) {
    if (corners.size() != 4) return false;
    sorted.clear();
    for (const auto &marker : corners) {
        if (marker.size() != 4) return false;
        cv::Point2f center(0.f, 0.f);
        for (const auto &point : marker) center += point;
        sorted.push_back(center * 0.25f);
    }
    std::sort(sorted.begin(), sorted.end(), [](const cv::Point2f &a, const cv::Point2f &b) { return a.y < b.y; });
    if (sorted[0].x > sorted[1].x) std::swap(sorted[0], sorted[1]);
    if (sorted[2].x > sorted[3].x) std::swap(sorted[2], sorted[3]);
    return true;
}

int main() {
    // 1. Initialize Valeria's Kinematics & Dynamixel Control
    PongRobot robot(5.0, 5.5, 6.3, 1, 2, 3, 4);
    if (!robot.init()) {
        std::cerr << "[-] Failed to initialize Dynamixel motors. Check USB permissions." << std::endl;
        return -1;
    }

    const int FIXED_SHOULDER = 341;
    const double GAMMA_PARALLEL = 0.0;
    
    std::cout << "[+] Moving to home defense position..." << std::endl;
    robot.moveToPlanar(0.0, 13.5, GAMMA_PARALLEL, FIXED_SHOULDER);

    // 2. Load Vision Parameters
    int loH = 0, hiH = 0, loS = 0, hiS = 0, loV = 0, hiV = 0;
    if (!readColorParameters(loH, hiH, loS, hiS, loV, hiV)) {
        std::cerr << "[-] Error: Could not read color parameters." << std::endl;
        return -1;
    }

    cv::Mat cameraMatrix, distortion;
    bool calibrated = readCameraParameters(cameraMatrix, distortion);

    cv::VideoCapture camera(2, cv::CAP_V4L2);
    if (!camera.isOpened()) {
        std::cerr << "[-] Error: Cannot open webcam on index 2." << std::endl;
        return -1;
    }

    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    cv::Ptr<cv::aruco::DetectorParameters> params = cv::aruco::DetectorParameters::create();
    cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    bool motorsArmed = false;
    bool homographyReady = false;
    cv::Mat homography;
    auto last_motor_update = std::chrono::steady_clock::now();

    std::cout << "\n=======================================================" << std::endl;
    std::cout << "  AUTONOMOUS VISUAL SERVOING ACTIVE                    " << std::endl;
    std::cout << "  - Show 4 ArUco markers to map the table              " << std::endl;
    std::cout << "  - Press 'M' to arm/disarm robot movement             " << std::endl;
    std::cout << "  - Press 'ESC' to exit                                " << std::endl;
    std::cout << "=======================================================" << std::endl;

    while (true) {
        cv::Mat frame;
        if (!camera.read(frame) || frame.empty()) break;

        if (calibrated) {
            cv::Mat corrected;
            cv::undistort(frame, corrected, cameraMatrix, distortion);
            frame = corrected;
        }
        
        cv::Mat screen = frame.clone();

        // ArUco Table Calibration
        if (!homographyReady) {
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            cv::aruco::detectMarkers(frame, dictionary, markerCorners, markerIds, params);
            
            if (!markerIds.empty()) {
                cv::aruco::drawDetectedMarkers(screen, markerCorners, markerIds);
            }

            if (markerIds.size() == 4) {
                std::vector<cv::Point2f> centers;
                if (sortedArUcoCenters(markerCorners, centers)) {
                    homography = cv::getPerspectiveTransform(centers, WORLD_MARKERS);
                    homographyReady = true;
                    std::cout << "[+] Table Mapped! ArUco Homography locked." << std::endl;
                }
            }
        }

        // Ball Color Detection
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(loH, loS, loV), cv::Scalar(hiH, hiS, hiV), mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        bool ballDetected = false;
        cv::Point2f pixel(0.f, 0.f);
        double bestArea = 0;

        for (const auto& contour : contours) {
            double area = cv::contourArea(contour);
            if (area >= MIN_BALL_PIXELS) {
                double perimeter = cv::arcLength(contour, true);
                if (perimeter > 0) {
                    double circularity = 4.0 * M_PI * area / (perimeter * perimeter);
                    
                    if (circularity > 0.70 && area > bestArea) {
                        cv::Moments m = cv::moments(contour);
                        pixel = cv::Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
                        bestArea = area;
                        ballDetected = true;
                    }
                }
            }
        }

        bool inReach = false;

        if (homographyReady && ballDetected) {
            cv::circle(screen, pixel, 9, cv::Scalar(0, 255, 0), 2);

            std::vector<cv::Point2f> world;
            cv::perspectiveTransform(std::vector<cv::Point2f>{pixel}, world, homography);

            if (world.size() == 1) {
                // Coordenadas del mundo (Cancha)
                float ball_X = world[0].x; // Eje largo de la mesa
                float ball_Y = world[0].y; // Eje ancho de la mesa

                // CORRECCIÓN: Mapeo exacto a tu Cinemática (Valeria's IK)
                // Tu robot tiene la "Y" hacia adelante y la "X" a los lados.
                // Si el robot está a 24 cm del centro, la distancia hacia adelante es 24 - posición_bola
                double target_y = ROBOT_X - ball_X; 
                double target_x = ball_Y;           

                // Safety Clamp para proteger los límites físicos
                target_x = std::clamp(target_x, -4.2, 4.2);
                target_y = std::clamp(target_y, 11.5, 16.5);
                inReach = true;

                // Log para ver exactamente qué estamos mandando
                std::cout << "[TARGET] Bola en Cancha(X,Y): " << ball_X << ", " << ball_Y 
                          << " | Enviando al Brazo(Lat, Fwd): " << target_x << ", " << target_y << std::endl;

                if (motorsArmed) {
                    auto now = std::chrono::steady_clock::now();
                    if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_motor_update).count() >= 20) {
                        robot.moveToPlanar(target_x, target_y, GAMMA_PARALLEL, FIXED_SHOULDER);
                        last_motor_update = now;
                    }
                }
            }
        }

        std::string status = motorsArmed ? "ARMED (Moving)" : "DISARMED (Press M)";
        if (!homographyReady) status = "Show 4 ArUco Markers";
        
        cv::putText(screen, "Status: " + status, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 255), 2);
        
        cv::imshow("PongBot - Visual Servoing", screen);
        cv::imshow("Ball Mask", mask);

        int key = cv::waitKey(static_cast<int>(1000.0 / FPS));
        if (key == 27) break; 
        if (key == 'm' || key == 'M') {
            motorsArmed = !motorsArmed;
            if (!motorsArmed) robot.moveToPlanar(0.0, 13.5, GAMMA_PARALLEL, FIXED_SHOULDER); 
        }
    }

    camera.release();
    cv::destroyAllWindows();
    return 0;
}
