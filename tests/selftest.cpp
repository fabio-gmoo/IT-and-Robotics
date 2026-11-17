// =============================================================================
//  selftest: comprobaciones SIN hardware.   make test
//   1. El YAML se carga.
//   2. La IK nueva da EXACTAMENTE los mismos ticks que pong_robot.hpp (Valeria).
//   3. El smash nunca pide un punto fuera de alcance en toda la franja lateral.
//   4. La predicción con rebotes da el resultado esperado.
//   5. Partidas simuladas IA vs IA y IA sola: la IA devuelve la bola.
// =============================================================================
#include <cmath>
#include <cstdio>
#include <iostream>

#include "pong_robot.hpp"  // legacy/pong_robot_valeria (referencia)
#include "pongbot/arm.hpp"
#include "pongbot/config.hpp"
#include "pongbot/controllers.hpp"
#include "pongbot/sim.hpp"
#include "pongbot/vision.hpp"

using namespace pongbot;

static int failures = 0;
#define CHECK(cond, msg)                                         \
    do {                                                         \
        if (cond) std::cout << "  [OK]   " << msg << "\n";       \
        else { std::cout << "  [FAIL] " << msg << "\n"; ++failures; } \
    } while (0)

// Simula una partida de 'seconds' sin GUI. Devuelve devoluciones (rebotes en pala).
static int simulateMatch(const Config& cfg, bool twoArms, double seconds, int& goalsA, int& goalsB) {
    PongArm a(cfg.arms[0], true), b(cfg.arms[1], true);
    // Reloj virtual: la simulación va mucho más rápido que el tiempo real
    const Clock::time_point base = Clock::now();
    double vt = 0.0;
    auto vclock = [&] { return base + std::chrono::microseconds(static_cast<long>(vt * 1e6)); };
    a.setClock(vclock);
    b.setClock(vclock);
    std::string e;
    a.connect(e); b.connect(e);
    AIController ca(true), cb(true);
    TableVision vis(cfg.vision);
    SimTable sim(cfg.vision);
    goalsA = goalsB = 0;
    int returns = 0;
    double lastVx = 0;
    const double dt = 1.0 / 200.0;  // bucle de control a 200 Hz
    double nextFrame = 0;
    bool wasVisible = true;
    int smashes = 0;
    for (double t = 0; t < seconds; t += dt) {
        vt = t;
        const bool wasSmashing = a.isSmashing() || b.isSmashing();
        std::vector<const PongArm*> playing = {&a};
        if (twoArms) playing.push_back(&b);
        sim.step(dt, playing);
        if (t >= nextFrame) {  // la cámara solo ve a 30 FPS
            vis.injectBall(sim.visible(), sim.pos(), t);
            nextFrame += 1.0 / 30.0;
            ca.update(a, vis, 1.0 / 30.0);
            if (twoArms) cb.update(b, vis, 1.0 / 30.0);
        }
        a.update();
        if (twoArms) b.update();
        if (!wasSmashing && (a.isSmashing() || b.isSmashing())) ++smashes;
        // Contar devoluciones: cambio de signo de vx cerca de una pala
        const BallState& bs = vis.ball();
        if (bs.vel_valid && lastVx != 0 && (bs.vel.x > 0) != (lastVx > 0) && std::fabs(bs.pos.x) > 12) ++returns;
        if (bs.vel_valid) lastVx = bs.vel.x;
        if (wasVisible && !sim.visible()) {
            if (bs.pos.x > 0) ++goalsA; else ++goalsB;  // gol en el lado de A (+X) / B
        }
        wasVisible = sim.visible();
    }
    std::cout << "     smashes=" << smashes << "\n";
    return returns;
}

int main() {
    PongArm::verbose() = false;
    Config cfg;
    std::string err;
    std::cout << "1) Configuración\n";
    CHECK(loadConfig("config/pongbot.yaml", cfg, err), "config/pongbot.yaml se carga " + err);
    CHECK(cfg.arms.size() == 2, "2 brazos definidos");
    CHECK(cfg.joysticks.size() == 2, "2 joysticks definidos");

    std::cout << "2) IK idéntica a pong_robot.hpp (brazo A)\n";
    {
        PongRobot ref(5.0, 5.5, 6.3, 1, 2, 3, 4);
        PongArm arm(cfg.arms[0], true);
        int compared = 0, mismatches = 0, reach_mismatch = 0;
        for (double x = -8; x <= 8; x += 0.25)
            for (double y = 8; y <= 17; y += 0.25) {
                double r1 = 0, r2 = 0, r3 = 0, n1 = 0, n2 = 0, n3 = 0;
                const bool okRef = ref.computeIK(x, y, 0.0, r1, r2, r3);
                const bool okNew = arm.computeIK(x, y, 0.0, n1, n2, n3);
                if (okRef != okNew) { ++reach_mismatch; continue; }
                if (!okRef) continue;
                ++compared;
                if (ref.radToTicksBase(r1) != arm.jointToTicks(0, n1) ||
                    ref.radToTicksElbow(r2) != arm.jointToTicks(1, n2) ||
                    ref.radToTicksWrist(r3) != arm.jointToTicks(2, n3))
                    ++mismatches;
            }
        CHECK(reach_mismatch == 0, "mismo alcance en " + std::to_string(compared) + " puntos");
        CHECK(mismatches == 0, "mismos ticks en todos los puntos");
    }

    std::cout << "3) Smash siempre alcanzable\n";
    for (const auto& ac : cfg.arms) {
        PongArm arm(ac, true);
        bool homeOk = true, recoilOk = true;
        double minStrike = 1e9;
        for (double x = -ac.lateral_limit; x <= ac.lateral_limit + 1e-9; x += 0.1) {
            homeOk &= arm.reachable(x, ac.home_y);
            recoilOk &= arm.reachable(x, ac.recoil_y);
            minStrike = std::min(minStrike, arm.maxReachableY(x, ac.strike_y, ac.home_y));
        }
        CHECK(homeOk, "brazo " + ac.name + ": home_y alcanzable en todo ±lateral_limit");
        CHECK(recoilOk, "brazo " + ac.name + ": recoil_y alcanzable en todo ±lateral_limit");
        char buf[96];
        std::snprintf(buf, sizeof(buf), "brazo %s: strike_y efectivo mínimo = %.1f cm (> home)",
                      ac.name.c_str(), minStrike);
        CHECK(minStrike > ac.home_y + 0.5, buf);
    }

    std::cout << "4) Predicción con rebote\n";
    {
        TableVision vis(cfg.vision);
        vis.injectBall(true, {0.f, 0.f}, 0.0);
        vis.injectBall(true, {1.f, 1.f}, 0.1);   // v = (10, 10) cm/s
        vis.injectBall(true, {2.f, 2.f}, 0.2);
        double y, t;
        const bool ok = vis.predictIntercept(+1, 21.0, y, t);
        // sin paredes: y = 2 + 19 = 21 -> con paredes en ±8.15 => 2*8.15 - 21 = -4.7
        CHECK(ok && std::fabs(y - (-4.7)) < 0.05, "y predicha = " + std::to_string(y) + " (esperado -4.7)");
        CHECK(!vis.predictIntercept(-1, 21.0, y, t), "no predice para el brazo del que se aleja");
    }

    std::cout << "5) Partidas simuladas (60 s)\n";
    {
        int gA, gB;
        const int r2 = simulateMatch(cfg, true, 60.0, gA, gB);
        std::cout << "     IA vs IA: devoluciones=" << r2 << " goles en A=" << gA << " en B=" << gB << "\n";
        CHECK(r2 >= 10, "IA vs IA: la IA devuelve la bola");
        const int r1 = simulateMatch(cfg, false, 60.0, gA, gB);
        std::cout << "     IA sola: devoluciones=" << r1 << " goles en A=" << gA << "\n";
        CHECK(r1 >= 5, "IA sola contra la pared: devuelve la bola");
    }

    std::cout << (failures == 0 ? "\nTODO OK\n" : "\nHAY FALLOS\n");
    return failures == 0 ? 0 : 1;
}
