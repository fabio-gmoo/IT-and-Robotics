// =============================================================================
//  Visión: cámara en un hilo aparte + homografía ArUco + detección de la bola
//  (HSV + circularidad) + estimación de velocidad en cm/s.
// =============================================================================
#ifndef PONGBOT_VISION_HPP
#define PONGBOT_VISION_HPP

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>

#include <opencv2/aruco.hpp>
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

#include "pongbot/config.hpp"

namespace pongbot {

// Lee la cámara continuamente en su propio hilo y guarda solo el último frame.
// Así el bucle de control nunca se queda esperando a la cámara.
class CameraThread {
public:
    ~CameraThread();
    // Intenta el índice configurado; si falla prueba 0..4 y avisa.
    bool start(const CameraConfig& cfg, std::string& info);
    void stop();
    // Copia el último frame si es más nuevo que last_id. Devuelve true si hay nuevo.
    bool latest(cv::Mat& frame, long& last_id);
    bool running() const { return running_; }
    int openedIndex() const { return opened_index_; }

private:
    void loop();
    cv::VideoCapture cap_;
    std::thread thread_;
    std::mutex mutex_;
    cv::Mat frame_;
    long frame_id_ = 0;
    std::atomic<bool> running_{false};
    int opened_index_ = -1;
};

struct BallState {
    bool detected = false;      // visto en este frame (píxeles)
    bool valid = false;         // visto y convertido a cm (hay homografía)
    cv::Point2f pixel;
    cv::Point2f pos;            // cm en coordenadas de mesa (X largo, Y ancho)
    cv::Point2f vel;            // cm/s
    bool vel_valid = false;
    double t = 0.0;             // segundos (reloj monotónico)
};

class TableVision {
public:
    explicit TableVision(const VisionConfig& cfg) : cfg_(cfg) {}

    bool loadColor(std::string& error);
    void loadCameraCalibration(const std::string& file, bool undistort);

    // Procesa un frame: undistort, ArUco (hasta fijar la homografía), bola.
    // 'debug' recibe el frame anotado.
    void process(const cv::Mat& frame, double t, cv::Mat& debug);
    // Para el simulador: inyecta directamente la posición de la bola en cm.
    void injectBall(bool visible, const cv::Point2f& pos, double t);

    void resetHomography();
    bool homographyReady() const { return homography_ready_; }
    const BallState& ball() const { return ball_; }
    const cv::Mat& mask() const { return mask_; }

    // Y de mesa donde la bola cruzará la línea u = side*X = target_u,
    // con rebotes en las paredes laterales. false si la bola no se acerca.
    bool predictIntercept(int side, double target_u, double& y_out, double& t_out) const;

private:
    void updateBall(bool visible, const cv::Point2f& pos, double t);

    VisionConfig cfg_;
    int loH_ = 0, hiH_ = 179, loS_ = 0, hiS_ = 255, loV_ = 0, hiV_ = 255;
    cv::Mat camMatrix_, distCoeffs_;
    bool undistort_ = false;
    cv::Mat map1_, map2_;

    bool homography_ready_ = false;
    cv::Mat H_;
    cv::Mat mask_;

    BallState ball_;
    double last_seen_t_ = -1.0;
    cv::Point2f last_pos_;
};

}  // namespace pongbot

#endif
