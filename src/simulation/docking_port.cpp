#include "docking_port.hpp"

#include "physics_body.hpp"
#include <cmath>
#include <numbers> 

// Координаты порта в мировой СК
sf::Vector2f DockingPort::world_position(const PhysicsBody& body) const {
    float c = std::cos(body.angle);   // косинус поворота тела
    float s = std::sin(body.angle);   // синус поворота тела

    // Поворачиваем localOffset на угол тела (матрица поворота):
    sf::Vector2f rotated = {
        local_offset.x * c - local_offset.y * s,  // x' = x·cos − y·sin
        local_offset.x * s + local_offset.y * c   // y' = x·sin + y·cos
    };

    // Прибавляем позицию центра масс в мире
    return body.position + rotated;
}

// Угол порта в мировой СК (смотрит наружу)
float DockingPort::world_angle(const PhysicsBody& body) const {
    // Мировой угол порта = угол тела + локальный угол порта.
    return body.angle + local_angle;
}

// Возвращает DockingTarget — всё, что нужно автопилоту.
DockingTarget compute_target(
    const DockingPort& station_port,
    const PhysicsBody& station,
    float approach_dist) const       // approachDist — сколько метров перед портом
{
    // Мировая позиция порта станции, финальная точка, куда должен прийти порт модуля.
    const sf::Vector2f port_position = station_port.world_position(station);

    // Требуемый мировой угол модуля: мировые углы портов отличаются на π.
    const float port_angle = station_port.world_angle(station) + std::numbers::pi_v<float>;

    // Точка промежуточного захода, направление «наружу» от порта станции - диничный вектор вдоль оси порта
    const float wa = station_port.world_angle(station);
    const sf::Vector2f port_dir = { std::cos(wa), std::sin(wa) };

    // approach_point — точка на оси порта, в approach_dist метрах от него.
    const sf::Vector2f approach_point = port_position + port_dir * approach_dist;

    return DockingTarget {
        .approach_point = approach_point,
        .port_position  = port_position,
        .port_angle     = port_angle
    };
}

// Проверяем стыковку
[[nodiscard]] DockingResult check_docking(
    const DockingPort& port_a, const PhysicsBody& body_a,
    const DockingPort& port_b, const PhysicsBody& body_b,
    float pos_threshold,
    float angle_threshold,
    float speed_threshold)
{
    // Расстояние между портами в мировых координатах
    const sf::Vector2f pos_a = port_a.world_position(body_a);
    const sf::Vector2f pos_b = port_b.world_position(body_b);
    const sf::Vector2f delta_pos = pos_a - pos_b;

    const float dist = std::hypot(delta_pos.x, delta_pos.y);

    // Угловое рассогласование (мировые углы портов отличаются на π - идеал)
    // normalize_angle приводит результат в [-π, π], иначе порог в 1° никогда не сработает
    const float wa = port_a.world_angle(body_a);
    const float wb = port_b.world_angle(body_b);
    const float angle_diff = std::abs(normalize_angle(wa - wb - std::numbers::pi_v<float>));

    // Относительная скорость тел 
    const sf::Vector2f rel_vel = body_a.velocity - body_b.velocity;
    const float speed = std::hypot(rel_vel.x, rel_vel.y);

    // Успех = все три условия одновременно
    const bool success = (dist        < pos_threshold)
                      && (angle_diff  < angle_threshold)
                      && (speed       < speed_threshold);

    return DockingResult {
        .success    = success,
        .speed      = speed,      // нужен Scene для проверки на "взрыв"
        .angle_diff = angle_diff  // нужен HUD для отображения рассогласования
    };
}