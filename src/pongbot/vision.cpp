#include "pongbot/vision.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace pongbot {

// ============================================================================
// CameraThread
// ============================================================================
CameraThread::~CameraThread() { stop(); }

bool CameraThread::start(const CameraConfig& cfg, std::string& info) {
    std::vector<int> candidates = {cfg.index};
    for (int i = 0; i <= 4; ++i)
        if (i != cfg.index) candidates.push_back(i);

    for (int idx : candidates) {
        if (cap_.open(idx, cv::CAP_V4L2)) {
            cv::Mat probe;
            if (cap_.read(probe) && !probe.empty()) {
                opened_index_ = idx;
                break;
            }
            cap_.release();
        }
    }
    if (opened_index_ < 0) {
        info = "No se pudo abrir ninguna cámara (índices 0-4).";
        return false;
    }
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, cfg.width);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, cfg.height);
    cap_.set(cv::CAP_PROP_FPS, cfg.fps);
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);  // menos latencia si el driver lo permite

    info = "Cámara abierta en el índice " + std::to_string(opened_index_);
    if (opened_index_ != cfg.index)
        info += " (OJO: en config pone " + std::to_string(cfg.index) +
                "; si es la webcam del portátil cambia camera.index)";
    running_ = true;
    thread_ = std::thread(&CameraThread::loop, this);
    return true;
}

void CameraThread::stop() {
    running_ = false;
    if (thread_.joinable()) thread_.join();
    if (cap_.isOpened()) cap_.release();
}

void CameraThread::loop() {
    cv::Mat f;
    while (running_) {
        if (!cap_.read(f) || f.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        f.copyTo(frame_);
        ++frame_id_;
    }
}

bool CameraThread::latest(cv::Mat& frame, long& last_id) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (frame_id_ == last_id || frame_.empty()) return false;
    frame_.copyTo(frame);
    last_id = frame_id_;
    return true;
}

// ============================================================================
// TableVision
// ============================================================================
bool TableVision::loadColor(std::string& error) {
    cv::FileStorage fs(cfg_.color_file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        error = "No se pudo leer " + cfg_.color_file + " (usa ./bin/hsv_tuner y pulsa S)";
        return false;
    }
    fs["lowH"] >> loH_; fs["highH"] >> hiH_;
    fs["lowS"] >> loS_; fs["highS"] >> hiS_;
    fs["lowV"] >> loV_; fs["highV"] >> hiV_;
    return true;
}

void TableVision::loadCameraCalibration(const std::string& file, bool undistort) {
    undistort_ = false;
    if (!undistort) return;
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[VISION] Sin calibración de cámara (" << file << "), sin undistort.\n";
        return;
    }
    fs["camera_matrix"] >> camMatrix_;
    fs["distortion_coefficients"] >> distCoeffs_;
    undistort_ = !camMatrix_.empty() && !distCoeffs_.empty();
}

void TableVision::resetHomography() {
    homography_ready_ = false;
    H_.release();
}

namespace {
// Ordena los centros por posición en la imagen: TL, TR, BL, BR
// (igual que sortedArUcoCenters de RobotServoing).
bool sortedArUcoCenters(const std::vector<std::vector<cv::Point2f>>& corners,
                        std::vector<cv::Point2f>& sorted) {
    if (corners.size() != 4) return false;
    sorted.clear();
    for (const auto& marker : corners) {
        if (marker.size() != 4) return false;
        cv::Point2f c(0.f, 0.f);
        for (const auto& p : marker) c += p;
        sorted.push_back(c * 0.25f);
    }
    std::sort(sorted.begin(), sorted.end(),
              [](const cv::Point2f& a, const cv::Point2f& b) { return a.y < b.y; });
    if (sorted[0].x > sorted[1].x) std::swap(sorted[0], sorted[1]);
    if (sorted[2].x > sorted[3].x) std::swap(sorted[2], sorted[3]);
    return true;
}
}  // namespace

void TableVision::process(const cv::Mat& input, double t, cv::Mat& debug) {
    cv::Mat frame;
    if (undistort_) {
        if (map1_.empty() || map1_.size() != input.size())
            cv::initUndistortRectifyMap(camMatrix_, distCoeffs_, cv::Mat(), camMatrix_,
                                        input.size(), CV_16SC2, map1_, map2_);
        cv::remap(input, frame, map1_, map2_, cv::INTER_LINEAR);
    } else {
        frame = input;
    }
    debug = frame.clone();

    // 1) Homografía: se calcula una vez con los 4 ArUco y se congela.
    if (!homography_ready_) {
        static const auto dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);
        static const auto params = cv::aruco::DetectorParameters::create();
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary, corners, ids, params);
        if (!ids.empty()) cv::aruco::drawDetectedMarkers(debug, corners, ids);
        std::vector<cv::Point2f> centers;
        if (ids.size() == 4 && sortedArUcoCenters(corners, centers)) {
            cv::Mat candidate = cv::getPerspectiveTransform(centers, cfg_.markers);
            if (!candidate.empty() && cv::checkRange(candidate)) {
                H_ = candidate.clone();
                homography_ready_ = true;
                std::cout << "[VISION] 4 ArUco detectados. Homografía fijada.\n";
            }
        }
    }

    // 2) Bola: HSV + morfología + mayor contorno suficientemente circular.
    static const cv::Mat element = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::Mat hsv;
    cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(loH_, loS_, loV_), cv::Scalar(hiH_, hiS_, hiV_), mask_);
    cv::morphologyEx(mask_, mask_, cv::MORPH_OPEN, element);
    cv::morphologyEx(mask_, mask_, cv::MORPH_CLOSE, element);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    bool found = false;
    double best = 0.0;
    cv::Point2f pixel;
    for (const auto& c : contours) {
        const double area = cv::contourArea(c);
        if (area < cfg_.min_ball_area) continue;
        const double perim = cv::arcLength(c, true);
        if (perim <= 0) continue;
        const double circ = 4.0 * M_PI * area / (perim * perim);
        if (circ < cfg_.min_circularity || area <= best) continue;
        const cv::Moments m = cv::moments(c);
        if (m.m00 <= 0) continue;
        pixel = cv::Point2f(static_cast<float>(m.m10 / m.m00), static_cast<float>(m.m01 / m.m00));
        best = area;
        found = true;
    }

    ball_.detected = found;
    ball_.pixel = pixel;
    cv::Point2f world;
    bool valid = false;
    if (found) cv::circle(debug, pixel, 10, cv::Scalar(0, 255, 0), 2);
    if (found && homography_ready_) {
        std::vector<cv::Point2f> out;
        cv::perspectiveTransform(std::vector<cv::Point2f>{pixel}, out, H_);
        if (out.size() == 1 && std::isfinite(out[0].x) && std::isfinite(out[0].y)) {
            world = out[0];
            if (cfg_.flip_x) world.x = -world.x;
            if (cfg_.flip_y) world.y = -world.y;
            valid = true;
        }
    }
    updateBall(valid, world, t);
}

void TableVision::injectBall(bool visible, const cv::Point2f& pos, double t) {
    ball_.detected = visible;
    updateBall(visible, pos, t);
}

void TableVision::updateBall(bool visible, const cv::Point2f& pos, double t) {
    ball_.valid = visible;
    ball_.t = t;
    if (!visible) {
        if (last_seen_t_ < 0 || t - last_seen_t_ > cfg_.lost_timeout_s) ball_.vel_valid = false;
        return;
    }
    if (last_seen_t_ >= 0 && t - last_seen_t_ <= cfg_.lost_timeout_s) {
        const double dt = t - last_seen_t_;
        if (dt > 1e-3) {
            cv::Point2f v = (pos - last_pos_) * static_cast<float>(1.0 / dt);
            const float a = static_cast<float>(std::clamp(cfg_.velocity_alpha, 0.0, 1.0));
            ball_.vel = ball_.vel_valid ? (a * v + (1.f - a) * ball_.vel) : v;
            ball_.vel_valid = true;
        }
    } else {
        ball_.vel = cv::Point2f(0, 0);
        ball_.vel_valid = false;
    }
    ball_.pos = pos;
    last_pos_ = pos;
    last_seen_t_ = t;
}

bool TableVision::predictIntercept(int side, double target_u, double& y_out, double& t_out) const {
    if (!ball_.valid || !ball_.vel_valid) return false;
    const double u = side * ball_.pos.x;
    const double vu = side * ball_.vel.x;
    if (vu <= 1.0) return false;                 // no se acerca (o casi parada)
    const double t = (target_u - u) / vu;
    if (t < 0.0 || t > 2.0) return false;        // ya pasó o demasiado lejos
    // Rebotes en las paredes laterales: "desdoblamos" la trayectoria.
    const double W = cfg_.table_half_width;
    double y = ball_.pos.y + ball_.vel.y * t;
    const double period = 4.0 * W;
    double s = std::fmod(y + W, period);
    if (s < 0) s += period;
    y = (s <= 2.0 * W) ? (s - W) : (3.0 * W - s);
    y_out = y;
    t_out = t;
    return true;
}

}  // namespace pongbot
