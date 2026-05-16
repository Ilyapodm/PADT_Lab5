#pragma once

#include <SFML/System/Vector2.hpp>
#include <numbers>   // std::numbers::pi_v
#include <cmath>     // std::cos, std::sin, std::hypot


struct PhysicsBody {
    // Кинематика
    sf::Vector2f position = {};  // мировые координаты центра масс, м
    sf::Vector2f velocity = {};  // скорость центра масс, м/с

    float angle           = 0.f;   // угол поворота тела, рад
    float angular_vel     = 0.f;   // угловая скорость, рад/с

    // Накопители сил (сбрасываются каждый тик)
    sf::Vector2f net_force = {};   // суммарная сила на этот тик, Н
    float net_torque       = 0.f;  // суммарный момент сил, Н·м

    // Геометрия корпуса (OBB-коллизия)
    float width  = 10.f;  // м, по локальной оси X
    float height = 20.f;  // м, по локальной оси Y

    // Геттеры
    [[nodiscard]] float mass()    const noexcept { return mass_; }
    [[nodiscard]] float inertia() const noexcept { return inertia_; }

    // Сеттеры
    void set_mass(float m);          // пересчитывает inertia_ автоматически
    void set_size(float w, float h); // обновляет width, height и inertia_

    // Применение сил
    // Сила через центр масс — только линейное ускорение, торка нет
    void apply_force(sf::Vector2f f) noexcept {
        net_force += f;
    }

    // Сила в произвольной точке — и линейное ускорение, и торк
    // offset — от центра масс в мировых координатах (уже повёрнутый)
    void apply_force_at_point(sf::Vector2f f, sf::Vector2f offset) noexcept {
        net_force  += f;
        net_torque += offset.x * f.y - offset.y * f.x;  // 2D cross product
    }

    // Интегратор — Symplectic Euler
    void integrate(float dt) noexcept {
        velocity    += (net_force  / mass_)    * dt;  // 1. скорости
        angular_vel += (net_torque / inertia_) * dt;

        position   += velocity    * dt;              // 2. позиции (новые скорости)
        angle      += angular_vel * dt;
    }

    // Сброс накопителей — вызывать ПОСЛЕ integrate(), не до
    void reset_forces() noexcept {
        net_force  = {};
        net_torque = 0.f;
    }

    // Скорость (модуль velocity) — для HUD и check_docking
    [[nodiscard]] float speed() const noexcept {
        return std::hypot(velocity.x, velocity.y);
    }

private:
    float mass_    = 1.f;  // масса, кг
    float inertia_ = 1.f;  // момент инерции, кг·м²
};

// TODO сделать коллизию 