#include "autopilot.hpp"
#include "../utils/math_utils.hpp"  // normalizeAngle
#include "../simulation/thruster.hpp"

#include <cassert>
#include <cmath>
#include <numbers>
#include <algorithm>  // std::clamp


// ─────────────────────────────────────────────────────────────────────────────
// init — привязка к двигателям и инициализация PID
// Вызывается один раз из Scene до первого update()
// ─────────────────────────────────────────────────────────────────────────────
void Autopilot::init(const Thruster& t,
                     AutopilotPIDConfig pid_cfg,
                     const AutopilotConfig& ap_cfg)
{
    thruster_ = &t;

    // Переводим все градусы в радианы
    ap_cfg.align_to_approach_angle  = deg2rad(ap_cfg.align_to_approach_angle);
    ap_cfg.approach_to_align_angle  = deg2rad(ap_cfg.approach_to_align_angle);
    ap_cfg.approach_to_final_angle  = deg2rad(ap_cfg.approach_to_final_angle);
    ap_cfg.final_to_approach_angle  = deg2rad(ap_cfg.final_to_approach_angle);
    ap_cfg.docked_angle             = deg2rad(ap_cfg.docked_angle);

    // сохраняем конфиг
    config_ = ap_cfg;

    // Кэшируем индексы двигателей — поиск по имени происходит O(1) один раз,
    // а не каждый тик через unordered_map
    id_main_      = t.id("main_fwd");
    id_rcs_left_  = t.id("rcs_left");
    id_rcs_right_ = t.id("rcs_right");
    id_bwd_left_  = t.id("rcs_bwd_left");
    id_bwd_right_ = t.id("rcs_bwd_right");
    id_rot_cwl_   = t.id("rcs_rot_cw_l");
    id_rot_cwr_   = t.id("rcs_rot_cw_r");
    id_rot_ccwl_  = t.id("rcs_rot_ccw_l");
    id_rot_ccwr_  = t.id("rcs_rot_ccw_r");

    // Создаём PID из конфига
    pid_theta_ = make_PID(pid_cfg.theta);
    pid_x_     = make_PID(pid_cfg.x);
    pid_y_     = make_PID(pid_cfg.y);
    damp_x_    = make_PID(pid_cfg.damp_x);
    damp_y_    = make_PID(pid_cfg.damp_y);
}

// ─────────────────────────────────────────────────────────────────────────────
// update — главный метод, вызывается каждый физический тик из Scene
// Порядок: сначала FSM (update_phase), потом управление (compute_thrust)
// ─────────────────────────────────────────────────────────────────────────────
ThrustCommand Autopilot::update(const PhysicsBody&  module,
                                const DockingTarget& target,
                                float                dt)
{
    assert(thruster_ != nullptr && "Autopilot::init() не был вызван!");

    // Терминальные фазы — двигатели выключены, ничего не считаем
    if (phase_ == Phase::DOCKED || phase_ == Phase::EXPLODED) {
        ThrustCommand cmd;
        cmd.throttles.assign(thruster_->count(), 0.f);
        return cmd;
    }

    // 1. FSM: переключаем фазу если нужно, сбрасываем PID
    update_phase(module, target);

    // 2. Управление: считаем сигналы, маппим на throttles
    return compute_thrust(module, target, dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// notify_docked / notify_failed — уведомления от Scene
// Scene сама решает факт стыковки через checkDocking(), мы только меняем фазу
// ─────────────────────────────────────────────────────────────────────────────
void Autopilot::notify_docked()
{
    phase_ = Phase::DOCKED;
    // Сбрасываем все PID — они больше не нужны
    pid_theta_.reset(); pid_x_.reset(); pid_y_.reset();
    damp_x_.reset();    damp_y_.reset();
}

void Autopilot::notify_failed()
{
    phase_ = Phase::EXPLODED;
    pid_theta_.reset(); pid_x_.reset(); pid_y_.reset();
    damp_x_.reset();    damp_y_.reset();
}

// ─────────────────────────────────────────────────────────────────────────────
// Геттеры
// ─────────────────────────────────────────────────────────────────────────────
Phase Autopilot::phase() const { return phase_; }

AutopilotDebugInfo Autopilot::debug_info() const
{
    return {
        .pid_theta = is_pid_theta_active(),
        .pid_x     = is_pid_x_active(),
        .pid_y     = is_pid_y_active(),
        .damp_x    = is_damp_x_active(),
        .damp_y    = is_damp_y_active(),
    };
}

// ─────────────────────────────────────────────────────────────────────────────
// update_phase — FSM с таблицей переходов из SPEC
//
// Таблица переходов:
//  ALIGN    → APPROACH : |dθ| < 4°
//  APPROACH → ALIGN    : |dθ| > 8°                    (сбрасываем pid_x_)
//  APPROACH → FINAL    : |dX| < 50м && |dθ| < 3°      (сбрасываем damp_y_)
//  FINAL    → APPROACH : |dX| > 70м || |dθ| > 6°      (сбрасываем pid_y_)
//
// Сброс позиционного PID — при ВЫХОДЕ из фазы, где он был активен
// ─────────────────────────────────────────────────────────────────────────────
void Autopilot::update_phase(const PhysicsBody&   module,
                              const DockingTarget& target)
{
    // Вычисляем текущие отклонения
    const float dtheta = std::abs(
        normalize_angle(target.port_angle - module.angle)
    );
    const float dx = std::abs(target.port_position.x - module.position.x);

    switch (phase_) {

    case Phase::ALIGN:
        if (dtheta < config_.align_to_approach_angle) {
            // Переходим в APPROACH, активируется X-PID
            // damp_x_/damp_y_ были активны в ALIGN — сбрасываем
            damp_x_.reset();
            phase_ = Phase::APPROACH;
        }
        break;

    case Phase::APPROACH:
        if (dtheta > config_.approach_to_align_angle) {
            // Угол уплыл — возврат в ALIGN
            // X-PID был активен в APPROACH — сбрасываем при выходе
            pid_x_.reset();
            phase_ = Phase::ALIGN;
        }
        else if (dx < config_.approach_to_final_dx &&
                 dtheta < config_.approach_to_final_angle)
        {
            // Вышли на ось и выровнялись — финальный заход
            // damp_y_ был активен в APPROACH — сбрасываем при выходе
            damp_y_.reset();
            phase_ = Phase::FINAL;
        }
        break;

    case Phase::FINAL:
        if (dx > config_.final_to_approach_dx ||
            dtheta > config_.final_to_approach_angle)
        {
            // Сбились — откат в APPROACH
            // Y-PID был активен в FINAL — сбрасываем при выходе
            pid_y_.reset();
            phase_ = Phase::APPROACH;
        }
        break;

    // Терминальные фазы — переходов нет
    case Phase::DOCKED:
    case Phase::EXPLODED:
        break;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_thrust — собирает ThrustCommand из сигналов осей
//
// Каждый compute_* возвращает скаляр в условных единицах силы.
// Маппинг: положительный сигнал → один двигатель, отрицательный → другой.
// clamp01 гарантирует throttle ∈ [0, 1].
// ─────────────────────────────────────────────────────────────────────────────
ThrustCommand Autopilot::compute_thrust(const PhysicsBody&   module,
                                         const DockingTarget& target,
                                         float                dt)
{
    ThrustCommand cmd;
    cmd.throttles.assign(thruster_->count(), 0.f);

    // Лямбда-хелпер: записывает throttle по ThrusterID в cmd
    auto set = [&](ThrusterID id, float val) {
        cmd.throttles[static_cast<std::size_t>(id)] = clamp01(val);
    };

    // ── θ-управление (активно в ALIGN, APPROACH, FINAL) ──────────────
    if (is_pid_theta_active()) {
        const float sig = compute_theta_control(module, target, dt);
        // sig > 0 → CW, sig < 0 → CCW
        if (sig > 0.f) {
            set(id_rot_cwl_,  sig);
            set(id_rot_cwr_,  sig);
        } else {
            set(id_rot_ccwl_, -sig);
            set(id_rot_ccwr_, -sig);
        }
    }

    // ── X-управление (APPROACH, FINAL) ───────────────────────────────
    if (is_pid_x_active()) {
        const float sig = compute_x_control(module, target, dt);
        // sig > 0 → двигаться вправо (+X), sig < 0 → влево (-X)
        if (sig > 0.f) set(id_rcs_right_, sig);
        else           set(id_rcs_left_,  -sig);
    }

    // ── Y-управление (FINAL) ─────────────────────────────────────────
    if (is_pid_y_active()) {
        const float sig = compute_y_control(module, target, dt);
        // Y↓: sig > 0 → двигаться вниз (к станции) → main_fwd
        //     sig < 0 → тормозить → bwd симметрично
        if (sig > 0.f) {
            set(id_main_, sig);
        } else {
            set(id_bwd_left_,  -sig);
            set(id_bwd_right_, -sig);
        }
    }

    // ── Velocity dampers ─────────────────────────────────────────────
    if (is_damp_x_active()) {
        const float sig = compute_damp_x(module, dt);
        if (sig > 0.f) set(id_rcs_right_, sig);
        else           set(id_rcs_left_,  -sig);
    }

    if (is_damp_y_active()) {
        const float sig = compute_damp_y(module, dt);
        if (sig > 0.f) set(id_main_,      sig);
        else {
            set(id_bwd_left_,  -sig);
            set(id_bwd_right_, -sig);
        }
    }

    return cmd;
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_theta_control
// error = нормализованная разница между нужным углом и текущим
// normalizeAngle обязателен: без него PID видит 350° вместо -10°
// ─────────────────────────────────────────────────────────────────────────────
float Autopilot::compute_theta_control(const PhysicsBody&   module,
                                        const DockingTarget& target,
                                        float                dt)
{
    const float error = normalizeAngle(target.portAngle - module.angle);
    return pid_theta_.update(error, dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_x_control
// APPROACH: цель — approachPoint (выровняться на оси стыковки, не врезаться в порт)
// FINAL:    цель — portPosition  (идём прямо к порту)
// ─────────────────────────────────────────────────────────────────────────────
float Autopilot::compute_x_control(const PhysicsBody&   module,
                                    const DockingTarget& target,
                                    float                dt)
{
    const sf::Vector2f goal = (phase_ == Phase::APPROACH)
        ? target.approachPoint
        : target.portPosition;

    const float error = goal.x - module.position.x;
    return pid_x_.update(error, dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_y_control
// Только FINAL. Цель — portPosition.y
// Y↓: error > 0 означает «нам нужно двигаться вниз» → main_fwd
// ─────────────────────────────────────────────────────────────────────────────
float Autopilot::compute_y_control(const PhysicsBody&   module,
                                    const DockingTarget& target,
                                    float                dt)
{
    const float error = target.portPosition.y - module.position.y;
    return pid_y_.update(error, dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_damp_x / compute_damp_y
// Setpoint = 0: гасим скорость до нуля.
// ki > 0 в конфиге позволяет скомпенсировать постоянный ветер.
// ─────────────────────────────────────────────────────────────────────────────
float Autopilot::compute_damp_x(const PhysicsBody& module, float dt)
{
    // error = 0 - velocity.x: если летим вправо, нужен импульс влево
    return damp_x_.update(-module.velocity.x, dt);
}

float Autopilot::compute_damp_y(const PhysicsBody& module, float dt)
{
    return damp_y_.update(-module.velocity.y, dt);
}