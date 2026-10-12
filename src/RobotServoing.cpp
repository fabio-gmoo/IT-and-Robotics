#include <algorithm>
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

// All lengths and world positions are in centimetres.
constexpr float L1 = 5.0f;
constexpr float L2 = 6.0f;
constexpr float ROBOT_X = 38.0f;
constexpr float ROBOT_Y = 0.0f;
constexpr int MIN_BALL_PIXELS = 150;
constexpr int MIN_CONSECUTIVE_FRAMES = 3;
constexpr int FRAMES_TO_REARM = 5;
constexpr double FPS = 30.0;

const std::string DXL_PORT = "/dev/ttyUSB0";
const std::string COLOR_FILE = "./data/color_params_data.xml";
const std::string CAMERA_FILE = "./data/camera_calibration_params.xml";

// New measured ArUco centre coordinates, 1 cm closer to the centre.
// Order corresponds to top-left, top-right, bottom-left, bottom-right
// AS SEEN BY THE CAMERA. Check this orientation before using motors.
const std::vector<cv::Point2f> WORLD_MARKERS = {
    {-20.4f,  8.15f}, {20.4f,  8.15f},
    {-20.4f, -8.15f}, {20.4f, -8.15f}
};

bool readColorParameters(int &loH, int &hiH, int &loS,
                         int &hiS, int &loV, int &hiV) {
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

// Ordering by camera image position. For a rotated camera or other markers,
// use explicit ArUco IDs instead of sorting by screen position.
bool sortedArUcoCenters(const std::vector<std::vector<cv::Point2f>> &corners,
                        std::vector<cv::Point2f> &sorted) {
    if (corners.size() != 4) return false;
    sorted.clear();
    for (const auto &marker : corners) {
        if (marker.size() != 4) return false;
        cv::Point2f center(0.f, 0.f);
        for (const auto &point : marker) center += point;
        sorted.push_back(center * 0.25f);
    }
    std::sort(sorted.begin(), sorted.end(),
              [](const cv::Point2f &a, const cv::Point2f &b) {
                  return a.y < b.y;
              });
    if (sorted[0].x > sorted[1].x) std::swap(sorted[0], sorted[1]);
    if (sorted[2].x > sorted[3].x) std::swap(sorted[2], sorted[3]);
    return true;
}

int main() {
    int loH = 0, hiH = 0, loS = 0, hiS = 0, loV = 0, hiV = 0;
    if (!readColorParameters(loH, hiH, loS, hiS, loV, hiV)) {
        std::cerr << "[ERROR] Could not read HSV values in " << COLOR_FILE << "\n";
        return 1;
    }
    cv::Mat cameraMatrix, distortion;
    const bool calibrated = readCameraParameters(cameraMatrix, distortion);

    if (access(DXL_PORT.c_str(), R_OK | W_OK) != 0) {
        std::cerr << "[ERROR] Cannot access " << DXL_PORT
                  << "; check VirtualBox USB and dialout.\n";
        return 1;
    }
    cv::VideoCapture camera(0, cv::CAP_V4L2);
    if (!camera.isOpened()) {
        std::cerr << "[ERROR] Cannot open the webcam.\n";
        return 1;
    }

    DynamixelHandler dxl;
    dxl.setDeviceName(DXL_PORT);
    dxl.setProtocolVersion(2.0f);
    dxl.openPort();
    dxl.setBaudRate(1000000);
    // No torque until manual arming: prevents an unexpected first movement.
    // Note: verify physical joint limits and motor directions before pressing M.

    const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
    const auto params = cv::aruco::DetectorParameters::create();
    const auto element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));

    bool motorsArmed = false;
    bool targetSent = false;
    int inReachFrames = 0;
    int outsideFrames = 0;
    bool undistort = calibrated;
    // Capture the ArUco homography ONCE during startup and reuse it.
    bool homographyReady = false;
    cv::Mat homography;
    unsigned long frameIndex = 0;
    float qpen = deg2rad(-90);

    std::cout << "[INFO] Robot base = (" << ROBOT_X << ", " << ROBOT_Y << ") cm.\n"
              << "[INFO] Waiting for four ArUco markers once. Their map will then be frozen.\n"
              << "[INFO] M = arm/disarm motors, C = recalibrate (motors disarmed),\n"
              << "       U = undistort (only before calibration), ESC = exit.\n";

    while (true) {
        cv::Mat frame;
        if (!camera.read(frame) || frame.empty()) {
            std::cerr << "[ERROR] Camera frame unavailable.\n";
            break;
        }
        if (undistort) {
            cv::Mat corrected;
            cv::undistort(frame, corrected, cameraMatrix, distortion);
            frame = corrected;
        }
        cv::Mat screen = frame.clone();
        frameIndex++;

        // 1) Initial calibration: as soon as four markers are detected,
        //    save the homography and stop requiring their visibility.
        //    This is the equivalent of calibrating from a startup snapshot.
        if (!homographyReady) {
            std::vector<int> markerIds;
            std::vector<std::vector<cv::Point2f>> markerCorners;
            cv::aruco::detectMarkers(frame, dictionary, markerCorners,
                                     markerIds, params);
            if (!markerIds.empty())
                cv::aruco::drawDetectedMarkers(screen, markerCorners, markerIds);

            if (markerIds.size() == 4) {
                std::vector<cv::Point2f> centers;
                if (sortedArUcoCenters(markerCorners, centers)) {
                    cv::Mat candidate = cv::getPerspectiveTransform(
                        centers, WORLD_MARKERS);
                    if (!candidate.empty() && cv::checkRange(candidate)) {
                        homography = candidate.clone();
                        homographyReady = true;
                        // Also save the startup calibration frame for inspection.
                        try {
                            if (cv::imwrite("./data/aruco_calibration_snapshot.png", screen))
                                std::cout << "[CALIB] Saved snapshot: ./data/aruco_calibration_snapshot.png\n";
                        } catch (const cv::Exception &e) {
                            std::cerr << "[WARN] Could not save calibration image: " << e.what() << '\n';
                        }
                        std::cout << "[CALIB] 4 ArUco found. Homography frozen. "
                                  << "Markers may now leave the image.\n";
                    }
                }
            }
        }

        // 2) Detect the ball on the unannotated camera frame.
        cv::Mat hsv, mask;
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        cv::inRange(hsv, cv::Scalar(loH, loS, loV),
                    cv::Scalar(hiH, hiS, hiV), mask);
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, element);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, element);
        cv::Moments m = cv::moments(mask, true);
        bool ballDetected = m.m00 >= MIN_BALL_PIXELS;

        float xWorld = 0, yWorld = 0, xRobot = 0, yRobot = 0;
        float distance = 0;
        bool inReach = false;

        if (homographyReady && ballDetected) {
            cv::Point2f pixel(static_cast<float>(m.m10 / m.m00),
                              static_cast<float>(m.m01 / m.m00));
            cv::circle(screen, pixel, 9, cv::Scalar(0, 255, 0), 2);

            std::vector<cv::Point2f> world;
            cv::perspectiveTransform(std::vector<cv::Point2f>{pixel},
                                     world, homography);
            if (world.size() == 1) {
                xWorld = -world[0].x;
                yWorld = -world[0].y;

                // World -> robot base coordinates (translation only).
                xRobot = xWorld - ROBOT_X;
                yRobot = yWorld - ROBOT_Y;
                distance = std::hypot(xRobot, yRobot);

                inReach = std::isfinite(xWorld) && std::isfinite(yWorld)
                       && std::isfinite(distance)
                       && distance >= std::fabs(L1 - L2)
                       && distance <= L1 + L2;

                if (frameIndex % 10 == 0) {
                    std::cout << "[BALL] world=(" << xWorld << ", " << yWorld
                              << ") cm; robot=(" << xRobot << ", " << yRobot
                              << ") cm; d=" << distance << " cm; "
                              << (inReach ? "IN REACH" : "OUT OF REACH") << '\n';
                }
            }
        }

        // 3) One motor command when the ball enters the reachable region.
        // No X<0 restriction anymore.
        if (inReach) {
            inReachFrames++;
            outsideFrames = 0;
        } else {
            inReachFrames = 0;
            outsideFrames++;
            if (outsideFrames >= FRAMES_TO_REARM) targetSent = false;
        }

        if (motorsArmed && homographyReady && inReach && !targetSent &&
            inReachFrames >= MIN_CONSECUTIVE_FRAMES) {
            std::vector<float> ik = computeInverseKinematics(xRobot, yRobot, L1, L2);
            if (ik.size() >= 3 && std::isfinite(ik[1]) && std::isfinite(ik[2])) {
                std::vector<float> target = {ik[1], qpen, ik[2]};
                std::cout << "[ROBOT] Motor command requested for robot frame: ("
                          << xRobot << ", " << yRobot << ") cm\n";
                dxl.sendTargetJointPosition(target);
                targetSent = true;
            } else {
                std::cerr << "[WARNING] No valid inverse kinematics solution.\n";
            }
        }

        std::string message;
        if (!homographyReady) message = "Waiting for initial 4 ArUco markers";
        else if (!ballDetected) message = "Waiting for ball";
        else if (!inReach) message = "Ball outside robot workspace";
        else if (!motorsArmed) message = "IN REACH - press M to arm motors";
        else if (targetSent) message = "Command issued - waiting for next entry";
        else message = "Ball reachable - ready";

        cv::putText(screen, message, cv::Point(10, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 255), 2);
        cv::putText(screen, "M: arm/disarm  C: recalibrate  ESC: exit",
                    cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX,
                    0.45, cv::Scalar(255, 255, 255), 1);
        cv::imshow("RobotServoing", screen);
        cv::imshow("Thresholded Image", mask);

        int key = cv::waitKey(static_cast<int>(1000.0 / FPS));
        if (key == 27) break;
        if (key == 'u' && calibrated) {
            if (!homographyReady) undistort = !undistort;
            else std::cout << "[WARN] Press C while disarmed before changing calibration.\n";
        }
        if (key == 'c') {
            if (motorsArmed) {
                std::cout << "[WARN] Disarm motors with M before recalibration.\n";
            } else {
                homography.release();
                homographyReady = false;
                targetSent = false;
                inReachFrames = 0;
                outsideFrames = 0;
                std::cout << "[CALIB] Cleared. Waiting for 4 ArUco markers.\n";
            }
        }
        if (key == 'm') {
            motorsArmed = !motorsArmed;
            dxl.enableTorque(motorsArmed);
            targetSent = false;
            inReachFrames = 0;
            std::cout << (motorsArmed ? "[INFO] MOTORS ARMED\n"
                                      : "[INFO] MOTORS DISARMED\n");
        }
    }

    dxl.enableTorque(false);
    dxl.closePort();
    camera.release();
    cv::destroyAllWindows();
    return 0;
}

