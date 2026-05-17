#pragma once

#include "physics_body.hpp"
#include "../utils/math_utils.hpp"

// Система координат: Y↓, X→ (SFML)
// Земля — в центре мировых координат {0, 0}

struct OrbitConfig {
    float orbit_radius   = 800.f;   // м — расстояние от Земли до станции
    float orbital_speed  = 60.f;    // м/с — скорость по орбите
    float mu             = orbital_speed * orbital_speed * orbit_radius;
    //    ↑ вычисляется автоматически, не задаётся вручную
};

class Gravity {
public:
    explicit Gravity(sf::Vector2f earth_center = {0.f, 0.f},
                     float earth_mass          = 5.972e24f,
                     float G                   = 6.674e-11f)
        : earth_center_{earth_center}
        , earth_mass_{earth_mass}
        , G_{G}
    {}


    [[nodiscard]] sf::Vector2f force_on(const PhysicsBody& body) const {
        sf::Vector2f delta = body.position - earth_center_; // вектор от Земли к телу

        float dist = length(delta);
        sf::Vector2f dir = delta / dist;                    // единичный вектор ОТ Земли
        float magnitude = (G_ * earth_mass_ * body.mass()) / (dist * dist);
        return -dir * magnitude;                            // притяжение К Земле
    }

private:
    sf::Vector2f earth_center_;
    float        earth_mass_;
    float        G_;
};