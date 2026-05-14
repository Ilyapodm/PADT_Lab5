#pragma once

#include <SFML/System/Vector2.hpp>
#include <vector>

#include "PID.hpp"
#include "../simulation/physics_body.hpp"
#include "../simulation/thruster.hpp"
#include "../simulation/docking_port.hpp"

// Конфиг для Автопилота
struct AutopilotConfig {
    // Пороги переходов между фазами
    float align_to_approach_angle   = 4.f;   // градусы
    float approach_to_align_angle   = 8.f;
    float approach_to_final_dx      = 50.f;  // метры
    float approach_to_final_angle   = 3.f;
    float final_to_approach_dx      = 70.f;
    float final_to_approach_angle   = 6.f;

    // Пороги успешной стыковки
    float docked_dist             = 0.5f;  // метры
    float docked_angle            = 1.f;   // градусы
    float docked_speed            = 0.1f;  // м/с

    // Максимальная скорость при контакте
    float maxDocking_speed        = 0.5f;  // м/с
};


// конфиг коэффициентов для Пид регуляторов Автопилота
struct AutopilotPIDConfig {
    PIDConfig theta = { 2.0f, 0.02f, 3.0f  };
    PIDConfig x     = { 0.8f, 0.01f, 1.5f  };
    PIDConfig y     = { 0.5f, 0.005f, 1.2f };
    PIDConfig damp_x = { 1.2f, 0.05f, 0.8f  };
    PIDConfig damp_y = { 1.2f, 0.05f, 0.8f  };
};

// выход автопилота
struct ThrustCommand {
    std::vector<float> throttles; // размер == thruster.count()
};

// FSM состояния
enum class Phase {
    ALIGN,
    APPROACH,
    FINAL,
    DOCKED,
    EXPLODED   // терминальный, без возврата
};

// структура для публичного геттера для рендеринга
struct AutopilotDebugInfo {
    bool pid_theta, pid_x, pid_y;
    bool damp_x, damp_y;
};

class Autopilot {
public:
    // Инициализация — привязка к двигателям, однократно
    void init(const Thruster& t, AutopilotPIDConfig pid_cfg, const AutopilotConfig& ap_cfg);

    // Главный метод — вызывается каждый физический тик из Scene
    [[nodiscard]] ThrustCommand update(const PhysicsBody& module,
                                       const DockingTarget& target,
                                       float dt);

    // Уведомления от Scene о внешних событиях
    void notify_docked();
    void notify_failed();

    // Геттер фазы — для HUD и Scene
    [[nodiscard]] Phase phase() const;

    // debug для рендеринга
    [[nodiscard]] AutopilotDebugInfo debug_info() const;  

private:
    // ── Зависимости ─────────────────────────────────────────────────
    const Thruster* thruster_ = nullptr;

    // ── Состояние FSM ───────────────────────────────────────────────
    Phase phase_ = Phase::ALIGN;

    // ── Цель стыковки ───────────────────────────────────────────────
    sf::Vector2f target_pos_;
    float        target_angle_ = 0.f;

    // ── PID регуляторы (позиция) ────────────────────────────────────
    PID pid_theta_, pid_x_, pid_y_;

    // ── Velocity dampers (скорость → 0) ────────────────────────────
    PID damp_x_, damp_y_;

    // ── Индексы двигателей (заполняются в init()) ───────────────────
    ThrusterID id_main_;
    ThrusterID id_rcs_left_,  id_rcs_right_;
    ThrusterID id_bwd_left_,  id_bwd_right_;
    ThrusterID id_rot_cwl_,   id_rot_cwr_;
    ThrusterID id_rot_ccwl_,  id_rot_ccwr_;

    //TODO сделать не тупое хранение конфига как поле, а его парсинг, так как есть ненужные части
    // ── Конфиг ───────────────────
    AutopilotConfig config_;

    // ── FSM ─────────────────────────────────────────────────────────
    // Проверяет условия переходов, переключает phase_,
    // вызывает reset() нужных PID/damper
    void update_phase(const PhysicsBody& module, const DockingTarget& target);

    // ── Control ─────────────────────────────────────────────────────
    // Считает ThrustCommand для текущей фазы
    [[nodiscard]] ThrustCommand compute_thrust(const PhysicsBody& module,
                                              const DockingTarget& target,
                                              float dt);

    // Управление по отдельным осям — вызываются из computeThrust()
    [[nodiscard]] float compute_theta_control(const PhysicsBody& module,
                                            const DockingTarget& target,
                                            float dt);

    [[nodiscard]] float compute_x_control(const PhysicsBody& module,
                                        const DockingTarget& target,
                                        float dt);

    [[nodiscard]] float compute_y_control(const PhysicsBody& module,
                                        const DockingTarget& target,
                                        float dt);

    // Velocity dampers — гашение остаточных скоростей
    [[nodiscard]] float compute_damp_x(const PhysicsBody& module, float dt);
    [[nodiscard]] float compute_damp_y(const PhysicsBody& module, float dt);

    // θ-PID активен во всех управляемых фазах (ALIGN, APPROACH, FINAL)
    [[nodiscard]] bool is_pid_theta_active()  const { return true; }

    // X-PID (боковое смещение) — активен когда нужно выходить на ось стыковки
    [[nodiscard]] bool is_pid_x_active()  const { return phase_ == Phase::APPROACH || phase_ == Phase::FINAL; }

    // Y-PID (продольное сближение) — только в финальном заходе
    [[nodiscard]] bool is_pid_y_active()  const { return phase_ == Phase::FINAL; }

    // dampX — гасит остаточную скорость по X когда X-PID не управляет осью
	[[nodiscard]] bool is_damp_x_active() const { return phase_ == Phase::ALIGN; }

    // dampY — гасит остаточную скорость по Y когда Y-PID не управляет осью
    [[nodiscard]] bool is_damp_y_active() const { return phase_ == Phase::ALIGN || phase_ == Phase::APPROACH; }
};