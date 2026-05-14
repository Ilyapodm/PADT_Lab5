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
                     const AutopilotPIDConfig& pid_cfg,
                     AutopilotConfig ap_cfg)
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
    
    const sf::Vector2f delta = target.port_position - module.position;
    const float cos_a = std::cos(target.port_angle);
    const float sin_a = std::sin(target.port_angle);
    const float dx = std::abs(delta.x * cos_a + delta.y * sin_a); // dot(delta, right)


    switch (phase_) {

    case Phase::ALIGN:
        if (dtheta < config_.align_to_approach_angle) {
            // Переходим в APPROACH, активируется X-PID
            // damp_y_ все еще активен
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
    const float error = normalize_angle(target.port_angle - module.angle);
    return pid_theta_.update(error, dt);
}

// C++20
// ─────────────────────────────────────────────────────────────────────────────
// compute_x_control — боковое смещение (перпендикуляр к оси захода)
//
// Проецируем вектор delta на правый вектор порта, а не берём мировой X.
// APPROACH: выравниваемся на approachPoint
// FINAL:    идём к portPosition
//
// Система координат SFML: Y↓, угол от +X по часовой.
// right = ( cos(port_angle),  sin(port_angle) ) — перпендикуляр к оси захода
// ─────────────────────────────────────────────────────────────────────────────
float Autopilot::compute_x_control(const PhysicsBody& module,
                                   const DockingTarget& target,
                                   float dt)
{
    const sf::Vector2f goal = (phase_ == Phase::APPROACH)
        ? target.approach_point
        : target.port_position;

    // Вектор от модуля до цели в мировых координатах
    const sf::Vector2f delta = goal - module.position;

    // Единичный вектор «вправо» в системе порта
    // (перпендикуляр к направлению захода, Y↓ SFML)
    const float cos_a = std::cos(target.port_angle);
    const float sin_a = std::sin(target.port_angle);
    const sf::Vector2f right{ cos_a, sin_a };

    // Проекция: насколько мы «сбоку» от оси стыковки
    const float error = delta.x * right.x + delta.y * right.y;

    return pid_x_.update(error, dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_y_control — продольное смещение (вдоль оси захода)
//
// Проецируем delta на вектор «вперёд» (к порту), а не берём мировой Y.
// Активен только в FINAL. Цель — portPosition.
//
// forward = ( -sin(port_angle), cos(port_angle) ) — направление к порту
// error > 0: нужно двигаться «к порту» → main_fwd
// error < 0: мы проскочили порт → bwd (торможение)
// ─────────────────────────────────────────────────────────────────────────────
float Autopilot::compute_y_control(const PhysicsBody& module,
                                   const DockingTarget& target,
                                   float dt)
{
    const sf::Vector2f delta = target.port_position - module.position;

    // Единичный вектор «вперёд» вдоль оси захода (к порту)
    const float cos_a = std::cos(target.port_angle);
    const float sin_a = std::sin(target.port_angle);
    const sf::Vector2f forward{ -sin_a, cos_a };

    // Проекция: насколько мы «далеко» от порта вдоль оси
    const float error = delta.x * forward.x + delta.y * forward.y;

    return pid_y_.update(error, dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// local_velocity — проецирует мировую скорость на локальные оси модуля
//
// SFML Y↓, angle от +X по часовой:
//   local_right   = ( cos_a,  sin_a)  — ось X модуля
//   local_forward = (-sin_a,  cos_a)  — ось Y модуля (к носу)
//
// Это умножение вектора velocity на транспонированную матрицу поворота:
//   | cos_a   sin_a | * | vx |
//   |-sin_a   cos_a |   | vy |
// ─────────────────────────────────────────────────────────────────────────────
static sf::Vector2f local_velocity(const PhysicsBody& module) noexcept
{
    const float cos_a = std::cos(module.angle);
    const float sin_a = std::sin(module.angle);
    return {
         module.velocity.x * cos_a + module.velocity.y * sin_a,  // боковая
        -module.velocity.x * sin_a + module.velocity.y * cos_a   // продольная
    };
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_damp_x — гасим боковую скорость (local X)
// ─────────────────────────────────────────────────────────────────────────────
float Autopilot::compute_damp_x(const PhysicsBody& module, float dt)
{
    const float local_vx = local_velocity(module).x;
    return damp_x_.update(-local_vx, dt);
}

// ─────────────────────────────────────────────────────────────────────────────
// compute_damp_y — гасим продольную скорость (local Y)
// ─────────────────────────────────────────────────────────────────────────────
float Autopilot::compute_damp_y(const PhysicsBody& module, float dt)
{
    const float local_vy = local_velocity(module).y;
    return damp_y_.update(-local_vy, dt);
}