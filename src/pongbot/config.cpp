#include "pongbot/config.hpp"

#include <iostream>
#include <opencv2/core/persistence.hpp>

namespace pongbot {

namespace {

template <typename T>
void readOr(const cv::FileNode& node, const char* key, T& value) {
    const cv::FileNode n = node[key];
    if (!n.empty()) n >> value;
}

void readBool(const cv::FileNode& node, const char* key, bool& value) {
    const cv::FileNode n = node[key];
    if (!n.empty()) value = static_cast<int>(n) != 0;
}

void readStr(const cv::FileNode& node, const char* key, std::string& value) {
    const cv::FileNode n = node[key];
    if (!n.empty()) value = static_cast<std::string>(n);
}

void readInt3(const cv::FileNode& node, const char* key, std::array<int, 3>& value) {
    const cv::FileNode n = node[key];
    if (n.empty() || !n.isSeq() || n.size() != 3) return;
    for (int i = 0; i < 3; ++i) value[i] = static_cast<int>(n[i]);
}

ArmConfig readArm(const cv::FileNode& n) {
    ArmConfig a;
    readStr(n, "name", a.name);
    readBool(n, "enabled", a.enabled);
    readStr(n, "port", a.port);
    readOr(n, "baudrate", a.baudrate);
    const cv::FileNode ids = n["ids"];
    if (!ids.empty() && ids.isSeq() && ids.size() == 4) {
        a.id_base = ids[0]; a.id_shoulder = ids[1];
        a.id_elbow = ids[2]; a.id_wrist = ids[3];
    }
    readOr(n, "shoulder_ticks", a.shoulder_ticks);
    readInt3(n, "dir", a.dir);
    readInt3(n, "offset", a.offset);
    readInt3(n, "tick_min", a.tick_min);
    readInt3(n, "tick_max", a.tick_max);
    const cv::FileNode links = n["links"];
    if (!links.empty() && links.isSeq() && links.size() == 3) {
        a.L1 = links[0]; a.L2 = links[1]; a.L3 = links[2];
    }
    readOr(n, "gamma", a.gamma);
    readOr(n, "side", a.side);
    readOr(n, "lateral_sign", a.lateral_sign);
    readOr(n, "lateral_center", a.lateral_center);
    readOr(n, "lateral_limit", a.lateral_limit);
    a.lateral_min = -a.lateral_limit;
    a.lateral_max = a.lateral_limit;
    readOr(n, "lateral_min", a.lateral_min);
    readOr(n, "lateral_max", a.lateral_max);
    readOr(n, "home_y", a.home_y);
    readOr(n, "recoil_y", a.recoil_y);
    readOr(n, "strike_y", a.strike_y);
    readOr(n, "recoil_ms", a.recoil_ms);
    readOr(n, "strike_ms", a.strike_ms);
    readOr(n, "return_ms", a.return_ms);
    readOr(n, "cooldown_ms", a.cooldown_ms);
    readOr(n, "paddle_u", a.paddle_u);
    readOr(n, "smash_trigger_u", a.smash_trigger_u);
    readOr(n, "smash_min_speed", a.smash_min_speed);
    readOr(n, "smash_lead_s", a.smash_lead_s);
    readOr(n, "smash_lateral_window", a.smash_lateral_window);
    readOr(n, "goal_u", a.goal_u);
    readOr(n, "min_send_ms", a.min_send_ms);
    readOr(n, "write_gap_us", a.write_gap_us);
    readOr(n, "max_lateral_speed", a.max_lateral_speed);
    a.side = (a.side >= 0) ? 1 : -1;
    a.lateral_sign = (a.lateral_sign >= 0) ? 1 : -1;
    return a;
}

JoystickConfig readJoystick(const cv::FileNode& n) {
    JoystickConfig j;
    readStr(n, "device", j.device);
    readOr(n, "channel", j.channel);
    readOr(n, "center", j.center);
    readOr(n, "deadzone_low", j.deadzone_low);
    readOr(n, "deadzone_high", j.deadzone_high);
    readOr(n, "smash_threshold", j.smash_threshold);
    readBool(n, "invert_x", j.invert_x);
    readOr(n, "max_speed", j.max_speed);
    return j;
}

}  // namespace

const ArmConfig* Config::arm(const std::string& name) const {
    for (const auto& a : arms)
        if (a.name == name) return &a;
    return nullptr;
}

bool loadConfig(const std::string& path, Config& cfg, std::string& error) {
    cv::FileStorage fs;
    try {
        if (!fs.open(path, cv::FileStorage::READ)) {
            error = "No se pudo abrir " + path;
            return false;
        }
    } catch (const cv::Exception& e) {
        error = "YAML inválido en " + path + ": " + e.what();
        return false;
    }

    const cv::FileNode cam = fs["camera"];
    if (!cam.empty()) {
        readOr(cam, "index", cfg.camera.index);
        readOr(cam, "width", cfg.camera.width);
        readOr(cam, "height", cfg.camera.height);
        readOr(cam, "fps", cfg.camera.fps);
        readStr(cam, "calibration_file", cfg.camera.calibration_file);
        readBool(cam, "undistort", cfg.camera.undistort);
    }

    const cv::FileNode vis = fs["vision"];
    if (!vis.empty()) {
        readStr(vis, "color_file", cfg.vision.color_file);
        readOr(vis, "min_ball_area", cfg.vision.min_ball_area);
        readOr(vis, "min_circularity", cfg.vision.min_circularity);
        readBool(vis, "flip_x", cfg.vision.flip_x);
        readBool(vis, "flip_y", cfg.vision.flip_y);
        readOr(vis, "table_half_width", cfg.vision.table_half_width);
        readOr(vis, "table_half_length", cfg.vision.table_half_length);
        readOr(vis, "velocity_alpha", cfg.vision.velocity_alpha);
        readOr(vis, "lost_timeout_s", cfg.vision.lost_timeout_s);
        readBool(vis, "predict", cfg.vision.predict);
        const cv::FileNode m = vis["markers"];
        if (!m.empty() && m.isSeq() && m.size() == 8) {
            cfg.vision.markers.clear();
            for (int i = 0; i < 4; ++i)
                cfg.vision.markers.emplace_back(static_cast<float>(m[2 * i]),
                                                static_cast<float>(m[2 * i + 1]));
        }
    }

    const cv::FileNode game = fs["game"];
    if (!game.empty()) {
        readOr(game, "points_to_win", cfg.game.points_to_win);
        readBool(game, "auto_goals", cfg.game.auto_goals);
        readOr(game, "goal_frames", cfg.game.goal_frames);
        readOr(game, "rearm_center_u", cfg.game.rearm_center_u);
        readStr(game, "scores_file", cfg.game.scores_file);
    }

    const cv::FileNode arms = fs["arms"];
    if (!arms.empty() && arms.isSeq()) {
        cfg.arms.clear();
        for (const auto& a : arms) cfg.arms.push_back(readArm(a));
    }
    if (cfg.arms.empty()) cfg.arms.push_back(ArmConfig{});

    const cv::FileNode joys = fs["joysticks"];
    if (!joys.empty() && joys.isSeq()) {
        cfg.joysticks.clear();
        for (const auto& j : joys) cfg.joysticks.push_back(readJoystick(j));
    }
    if (cfg.joysticks.empty()) cfg.joysticks.push_back(JoystickConfig{});

    // Validación básica: dos brazos en el mismo puerto no pueden compartir IDs.
    for (size_t i = 0; i < cfg.arms.size(); ++i)
        for (size_t k = i + 1; k < cfg.arms.size(); ++k) {
            const auto& a = cfg.arms[i];
            const auto& b = cfg.arms[k];
            if (!a.enabled || !b.enabled || a.port != b.port) continue;
            const int ia[4] = {a.id_base, a.id_shoulder, a.id_elbow, a.id_wrist};
            const int ib[4] = {b.id_base, b.id_shoulder, b.id_elbow, b.id_wrist};
            for (int x : ia)
                for (int y : ib)
                    if (x == y) {
                        error = "Los brazos " + a.name + " y " + b.name +
                                " están en el mismo puerto (" + a.port +
                                ") con el mismo ID " + std::to_string(x) +
                                ". Cambia los IDs o usa puertos distintos.";
                        return false;
                    }
        }
    return true;
}

}  // namespace pongbot
