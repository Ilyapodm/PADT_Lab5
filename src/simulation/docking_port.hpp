#pragma once

#include "physics_body.hpp"

// Класс порта
struct DockingPort {
    // Локальная система координат
    sf::Vector2f local_offset;   // смещение от центра масс тела, м
    float        local_angle;    // угол порта в локальной СК тела, рад

    // Позиция и угол в мировых координатах
    [[nodiscard]] sf::Vector2f world_position(const PhysicsBody& body) const;
    [[nodiscard]] float        world_angle   (const PhysicsBody& body) const;
};

// Порт станции как цель для модуля
struct DockingTarget {
    sf::Vector2f approach_point;  // точка за портом, откуда начинать заход
    sf::Vector2f port_position;   // мировые координаты порта станции
    float        port_angle;      // требуемый угол подхода (включает в себя Pi)
};

// Вычисление — в DockingPort:
[[nodiscard]] DockingTarget compute_target(
    const DockingPort& station_port,
    const PhysicsBody& station) const;

// Результат стыковки
struct DockingResult { bool success; float speed; float angle_diff; };

[[nodiscard]] DockingResult check_docking(
    const DockingPort& port_a, const PhysicsBody& body_a,
    const DockingPort& port_b, const PhysicsBody& body_b,
    float pos_threshold   = 0.5f,    // м
    float angle_threshold = 0.0175f, // рад ≈ 1°
    float speed_threshold = 0.1f     // м/с
);