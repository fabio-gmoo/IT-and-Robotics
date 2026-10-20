#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <unistd.h>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include "Kinematics.h"
#include "DynamixelHandler.h"

// Positions and lengths in centimetres.
constexpr float ROBOT_X = 38.0f;
constexpr float ROBOT_Y = 0.0f;
constexpr float DEFAULT_L1 = 6.0f;    // Display only, not validated for motion
constexpr float DEFAULT_L2 = 11.7f;   // Display only, not validated for motion
constexpr int MIN_BALL_PIXELS = 150;
constexpr int MIN_ARM_PIXELS = 50;
constexpr int MIN_CONSECUTIVE_FRAMES = 3;
constexpr float MIN_TARGET_CHANGE_CM = 0.8f;
constexpr int MIN_COMMAND_INTERVAL_MS = 800;
constexpr double FPS = 30.0;

const std::string DXL_PORT = "/dev/ttyUSB0";
const std::string BALL_COLOR_FILE = "./data/color_params_data.xml";
const std::string ARM_COLOR_FILE = "./data/color_params_data_arm.xml";
const std::string CAMERA_FILE = "./data/camera_calibration_params.xml";

// ArUco positions in world coordinates, shifted 1 cm toward centre.
// Order: top-left, top-right, bottom-left, bottom-right IN THE CAMERA IMAGE.
const std::vector<cv::Point2f> WORLD_MARKERS = {
    {-20.4f, 8.15f}, {20.4f, 8.15f},
    {-20.4f, -8.15f}, {20.4f, -8.15f}
};

struct HSVLimits {
    int lowH=0, highH=179, lowS=0, highS=255, lowV=0, highV=255;
};

bool readHSV(const std::string& path, HSVLimits& c) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    if (fs["lowH"].empty() || fs["highH"].empty() || fs["lowS"].empty() ||
        fs["highS"].empty() || fs["lowV"].empty() || fs["highV"].empty()) return false;
    fs["lowH"] >> c.lowH; fs["highH"] >> c.highH;
    fs["lowS"] >> c.lowS; fs["highS"] >> c.highS;
    fs["lowV"] >> c.lowV; fs["highV"] >> c.highV;
    return c.lowH >= 0 && c.lowH <= c.highH && c.highH <= 179 &&
           c.lowS >= 0 && c.lowS <= c.highS && c.highS <= 255 &&
           c.lowV >= 0 && c.lowV <= c.highV && c.highV <= 255;
}

bool readCameraParameters(cv::Mat& matrix, cv::Mat& dist) {
    cv::FileStorage fs(CAMERA_FILE, cv::FileStorage::READ);
    if (!fs.isOpened()) return false;
    fs["camera_matrix"] >> matrix;
    fs["distortion_coefficients"] >> dist;
    return !matrix.empty() && !dist.empty();
}

bool sortedArUcoCenters(const std::vector<std::vector<cv::Point2f>>& corners,
                        std::vector<cv::Point2f>& sorted) {
    if (corners.size() != 4) return false;
    sorted.clear();
    for (const auto& marker : corners) {
        if (marker.size() != 4) return false;
        cv::Point2f centre(0.f, 0.f);
        for (const auto& p : marker) centre += p;
        sorted.push_back(centre * 0.25f);
    }
    std::sort(sorted.begin(), sorted.end(),
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
    if (sorted[0].x > sorted[1].x) std::swap(sorted[0], sorted[1]);
    if (sorted[2].x > sorted[3].x) std::swap(sorted[2], sorted[3]);
    return true;
}

cv::Mat makeMask(const cv::Mat& hsv, const HSVLimits& c, const cv::Mat& element) {
    cv::Mat mask;
    cv::inRange(hsv, cv::Scalar(c.lowH, c.lowS, c.lowV),
                     cv::Scalar(c.highH, c.highS, c.highV), mask);
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
    return mask;
}

// Track the largest colour blob instead of the centre of all matching pixels.
bool findCentre(const cv::Mat& mask, double minPixels, cv::Point2f& pixel) {
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    double largest = minPixels;
    bool found = false;
    for (const auto& c : contours) {
        const double area = cv::contourArea(c);
        if (area < largest) continue;
        const cv::Moments m = cv::moments(c);
        if (m.m00 <= 0) continue;
        pixel = cv::Point2f(static_cast<float>(m.m10 / m.m00),
                            static_cast<float>(m.m01 / m.m00));
        largest = area;
        found = true;
    }
    return found;
}

bool toWorld(const cv::Point2f& pixel, const cv::Mat& H, cv::Point2f& world) {
    if (H.empty()) return false;
    std::vector<cv::Point2f> transformed;
    cv::perspectiveTransform(std::vector<cv::Point2f>{pixel}, transformed, H);
    if (transformed.size() != 1) return false;
    world = transformed.front();  // Preserve the coordinate convention of RobotServoing
    return std::isfinite(world.x) && std::isfinite(world.y);
}

// Simple top-down display of both measured colour markers.
cv::Mat drawPlan(bool ballValid, const cv::Point2f& ball,
                 bool armValid, const cv::Point2f& arm) {
    cv::Mat plan(460, 760, CV_8UC3, cv::Scalar(246, 246, 246));
    const float minX = -25.f, maxX = 48.f, minY = -22.f, maxY = 22.f;
    auto point = [&](cv::Point2f p) {
        return cv::Point(static_cast<int>(35 + (p.x - minX) * 690 / (maxX - minX)),
                         static_cast<int>(430 - (p.y - minY) * 390 / (maxY - minY)));
    };
    cv::line(plan, point(cv::Point2f(minX, 0)), point(cv::Point2f(maxX, 0)),
             cv::Scalar(155, 155, 155), 1);
    cv::line(plan, point(cv::Point2f(0, minY)), point(cv::Point2f(0, maxY)),
             cv::Scalar(155, 155, 155), 1);
    for (const auto& marker : WORLD_MARKERS)
        cv::drawMarker(plan, point(marker), cv::Scalar(110, 110, 110), cv::MARKER_CROSS, 12, 2);
    const cv::Point base = point(cv::Point2f(ROBOT_X, ROBOT_Y));
    cv::circle(plan, base, 8, cv::Scalar(0, 0, 0), -1);
    cv::putText(plan, "Robot base (38, 0)", base + cv::Point(-130, -14),
                cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 0), 1);
    if (ballValid) {
        cv::circle(plan, point(ball), 8, cv::Scalar(0, 0, 255), -1);
        cv::putText(plan, "BALL", point(ball) + cv::Point(10, 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 0, 170), 2);
    }
    if (armValid) {
        cv::circle(plan, point(arm), 8, cv::Scalar(255, 80, 0), -1);
        cv::putText(plan, "ARM TIP", point(arm) + cv::Point(10, -12),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(180, 60, 0), 2);
    }
    if (ballValid && armValid) {
        cv::line(plan, point(ball), point(arm), cv::Scalar(120, 120, 120), 1);
        const float gap = cv::norm(ball - arm);
        cv::putText(plan, "Ball / arm marker distance: " + std::to_string(gap) + " cm",
                    cv::Point(18, 24), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                    cv::Scalar(10, 10, 10), 2);
    }
    return plan;
}

int main(int argc, char** argv) {
    bool moveEnabled = false;
    bool l1Measured = false, l2Measured = false;
    float L1 = DEFAULT_L1, L2 = DEFAULT_L2;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--move") moveEnabled = true;
        else if ((arg == "--l1-cm" || arg == "--l2-cm") && i + 1 < argc) {
            float value = 0.f;
            try { value = std::stof(argv[++i]); }
            catch (...) { std::cerr << "[ERROR] Invalid length.\n"; return 1; }
            if (!std::isfinite(value) || value <= 0) return 1;
            if (arg == "--l1-cm") { L1 = value; l1Measured = true; }
            else { L2 = value; l2Measured = true; }
        } else {
            std::cerr << "Usage: ./bin/RobotServoing [--move --l1-cm VALUE --l2-cm VALUE]\n";
            return 1;
        }
    }
    if (moveEnabled && !(l1Measured && l2Measured)) {
        std::cerr << "[ERROR] Motor mode requires measured lengths for BOTH segments.\n";
        return 1;
    }

    HSVLimits ballHSV, armHSV;
    if (!readHSV(BALL_COLOR_FILE, ballHSV)) {
        std::cerr << "[ERROR] Invalid or missing ball HSV file: " << BALL_COLOR_FILE << '\n';
        return 1;
    }
    if (!readHSV(ARM_COLOR_FILE, armHSV)) {
        std::cerr << "[ERROR] Invalid or missing arm HSV file: " << ARM_COLOR_FILE
                  << "\nRun RedBallDetection with arm marker and save using S.\n";
        return 1;
    }

    cv::Mat cameraMatrix, distortion;
    const bool calibrated = readCameraParameters(cameraMatrix, distortion);
    bool undistort = calibrated;
    if (moveEnabled && access(DXL_PORT.c_str(), R_OK | W_OK) != 0) {
        std::cerr << "[ERROR] Cannot access " << DXL_PORT << '\n'; return 1;
    }
    cv::VideoCapture camera(0, cv::CAP_V4L2);
    if (!camera.isOpened()) {
        std::cerr << "[ERROR] Cannot open camera.\n"; return 1;
    }
    DynamixelHandler dxl;
    bool torqueEnabled = false;
    if (moveEnabled) {
        dxl.setDeviceName(DXL_PORT);
        dxl.setProtocolVersion(2.0f);
        if (!dxl.openPort() || !dxl.setBaudRate(1000000)) {
            std::cerr << "[ERROR] Cannot initialize Dynamixel serial connection.\n";
            camera.release(); return 1;
        }
    }

    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    const auto params = cv::aruco::DetectorParameters::create();
    const auto element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::Mat H;
    bool homographyReady = false;
    unsigned long frameIndex = 0;
    int inReachFrames = 0;
    cv::Point2f previousTarget(0, 0);
    bool previousTargetValid = false;
    const float qpen = deg2rad(-90);
    using Clock = std::chrono::steady_clock;
    auto lastCommandTime = Clock::now() - std::chrono::seconds(10);

    std::cout << "[INFO] Robot base (38,0) cm.\n"
              << "[INFO] BALL HSV: " << BALL_COLOR_FILE << '\n'
              << "[INFO] ARM HSV: " << ARM_COLOR_FILE << '\n'
              << "[INFO] " << (moveEnabled ? "MOTOR MODE" : "OBSERVATION ONLY (NO MOTOR COMMANDS)") << '\n'
              << "[INFO] Waiting for 4 ArUco once; then calibration freezes.\n"
              << "[INFO] C: recalibrate; ESC: quit.\n";

    while (true) {
        cv::Mat frame;
        if (!camera.read(frame) || frame.empty()) {
            std::cerr << "[ERROR] Camera frame unavailable.\n"; break;
        }
        if (undistort) {
            cv::Mat corrected;
            cv::undistort(frame, corrected, cameraMatrix, distortion);
            frame = corrected;
        }
        cv::Mat screen = frame.clone();
        ++frameIndex;

        if (!homographyReady) {
            std::vector<int> ids;
            std::vector<std::vector<cv::Point2f>> corners;
            cv::aruco::detectMarkers(frame, dictionary, corners, ids, params);
            if (!ids.empty()) cv::aruco::drawDetectedMarkers(screen, corners, ids);
            if (ids.size() == 4) {
                std::vector<cv::Point2f> centres;
                if (sortedArUcoCenters(corners, centres)) {
                    cv::Mat candidate = cv::getPerspectiveTransform(centres, WORLD_MARKERS);
                    if (!candidate.empty() && cv::checkRange(candidate)) {
                        H = candidate.clone();
                        homographyReady = true;
                        cv::imwrite("./data/aruco_calibration_snapshot.png", screen);
                        std::cout << "[CALIB] Four ArUco detected. Homography frozen.\n";
                    }
                }
            }
        }

        cv::Mat hsv;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::Mat ballMask = makeMask(hsv, ballHSV, element);
        cv::Mat armMask = makeMask(hsv, armHSV, element);
        cv::Point2f ballPixel, armPixel, ballWorld, armWorld;
        const bool ballDetected = findCentre(ballMask, MIN_BALL_PIXELS, ballPixel);
        const bool armDetected = findCentre(armMask, MIN_ARM_PIXELS, armPixel);
        bool ballValid = homographyReady && ballDetected && toWorld(ballPixel, H, ballWorld);
        bool armValid = homographyReady && armDetected && toWorld(armPixel, H, armWorld);

        // If colour ranges overlap and identify the same blob, don't claim arm tracking.
        if (ballValid && armValid && cv::norm(ballPixel - armPixel) < 8.f) {
            armValid = false;
            if (frameIndex % 30 == 0)
                std::cout << "[WARN] Arm colour overlaps ball detection; adjust HSV.\n";
        }

        if (ballDetected)
            cv::circle(screen, ballPixel, 10, cv::Scalar(0, 0, 255), 2);
        if (armDetected)
            cv::circle(screen, armPixel, 10, cv::Scalar(255, 90, 0), 2);
        if (armValid)
            cv::putText(screen, "ARM TIP", armPixel + cv::Point2f(10.f, -8.f),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(255, 90, 0), 2);

        float xRobot = 0.f, yRobot = 0.f, distance = 0.f;
        bool inReach = false;
        if (ballValid) {
            xRobot = ballWorld.x - ROBOT_X;
            yRobot = ballWorld.y - ROBOT_Y;
            distance = std::hypot(xRobot, yRobot);
            inReach = std::isfinite(distance) &&
                      distance >= std::fabs(L1 - L2) && distance <= L1 + L2;
        }
        if (frameIndex % 10 == 0) {
            if (ballValid)
                std::cout << "[BALL] world=(" << ballWorld.x << ", " << ballWorld.y
                          << ") cm, robot=(" << xRobot << ", " << yRobot << ") cm, d="
                          << distance << " cm; " << (inReach ? "IN REACH" : "OUT OF REACH") << '\n';
            if (armValid)
                std::cout << "[ARM TIP] world=(" << armWorld.x << ", " << armWorld.y
                          << ") cm, robot=(" << armWorld.x - ROBOT_X << ", "
                          << armWorld.y - ROBOT_Y << ") cm\n";
            if (ballValid && armValid)
                std::cout << "[DISTANCE] Ball to arm marker = "
                          << cv::norm(ballWorld - armWorld) << " cm\n";
        }

        if (inReach) ++inReachFrames;
        else inReachFrames = 0;

        // Original motor logic is unchanged; arm detection is observation only.
        if (moveEnabled && ballValid && inReach &&
            inReachFrames >= MIN_CONSECUTIVE_FRAMES) {
            const cv::Point2f targetXY(xRobot, yRobot);
            const float displacement = previousTargetValid
                ? cv::norm(targetXY - previousTarget) : MIN_TARGET_CHANGE_CM;
            const auto now = Clock::now();
            const bool delayElapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - lastCommandTime).count() >= MIN_COMMAND_INTERVAL_MS;
            if (delayElapsed && (!previousTargetValid || displacement >= MIN_TARGET_CHANGE_CM)) {
                std::vector<float> ik = computeInverseKinematics(xRobot, yRobot, L1, L2);
                if (ik.size() >= 3 && std::isfinite(ik[1]) && std::isfinite(ik[2])) {
                    if (!torqueEnabled) {
                        if (!dxl.enableTorque(true)) {
                            std::cerr << "[ERROR] Torque enable failed.\n";
                            dxl.enableTorque(false);
                            break;
                        }
                        torqueEnabled = true;
                    }
                    std::vector<float> target = {ik[1], qpen, ik[2]};
                    const bool accepted = dxl.sendTargetJointPosition(target);
                    if (!accepted) {
                        std::cerr << "[ERROR] Motor command rejected.\n";
                        dxl.enableTorque(false);
                        torqueEnabled = false;
                        break;
                    }
                    std::cout << "[ROBOT] Writes acknowledged (movement not verified).\n";
                    previousTarget = targetXY;
                    previousTargetValid = true;
                    lastCommandTime = now;
                }
            }
        }

        std::string message = !homographyReady ? "Waiting for 4 ArUco markers"
            : !ballValid ? "Waiting for ball"
            : !inReach ? "Ball outside calculated workspace"
            : !moveEnabled ? "IN REACH - OBSERVATION ONLY"
            : "MOTOR MODE";
        cv::putText(screen, message, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
        cv::putText(screen, armValid ? "Arm tip detected" : "Arm tip NOT detected",
                    cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(255, 90, 0), 2);
        cv::imshow("RobotServoing", screen);
        cv::imshow("Ball Mask", ballMask);
        cv::imshow("Arm Mask", armMask);
        cv::imshow("Plan XY (cm)", drawPlan(ballValid, ballWorld, armValid, armWorld));

        const int key = cv::waitKey(static_cast<int>(1000.0 / FPS));
        if (key == 27) break;
        if (key == 'u' && calibrated) {
            if (!homographyReady) undistort = !undistort;
            else std::cout << "[WARN] Press C to recalibrate before toggling undistortion.\n";
        }
        if (key == 'c') {
            if (torqueEnabled) { dxl.enableTorque(false); torqueEnabled = false; }
            H.release();
            homographyReady = false;
            inReachFrames = 0;
            previousTargetValid = false;
            std::cout << "[CALIB] Waiting for four ArUco again.\n";
        }
    }

    if (moveEnabled) {
        if (torqueEnabled) dxl.enableTorque(false);
        dxl.closePort();
    }
    camera.release();
    cv::destroyAllWindows();
    return 0;
}

