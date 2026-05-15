#pragma once

#include <cmath>
#include <numbers>
#include <algorithm>   // std::clamp
#include <SFML/System/Vector2.hpp>

// ─────────────────────────────────────────────────────────────────────────────
// Углы
// ─────────────────────────────────────────────────────────────────────────────

// Нормализация угла в [-π, π].
[[nodiscard]] constexpr float normalizeAngle(float a) noexcept {
    return std::remainder(a, 2.f * std::numbers::pi_v<float>);
}

[[nodiscard]] constexpr float deg2rad(float deg) noexcept {
    return deg * std::numbers::pi_v<float> / 180.f;
}

[[nodiscard]] constexpr float rad2deg(float rad) noexcept {
    return rad * 180.f / std::numbers::pi_v<float>;
}

// ─────────────────────────────────────────────────────────────────────────────
// Скалярные операции
// ─────────────────────────────────────────────────────────────────────────────

// clamp в произвольный диапазон [lo, hi]
template<typename T>
[[nodiscard]] constexpr T clamp(T v, T lo, T hi) noexcept {
    return std::clamp(v, lo, hi);
}

// clamp в [0, 1] — для throttle двигателей
[[nodiscard]] constexpr float clamp01(float v) noexcept {
    return std::clamp(v, 0.f, 1.f);
}

// clamp в [-1, 1] — для нормированных управляющих сигналов
[[nodiscard]] constexpr float clampSymmetric(float v) noexcept {
    return std::clamp(v, -1.f, 1.f);
}

// Линейная интерполяция: t=0 → a, t=1 → b
// Используется в Renderer для интерполяции между физическими тиками
[[nodiscard]] constexpr float lerp(float a, float b, float t) noexcept {
    return a + (b - a) * t;
}

// ─────────────────────────────────────────────────────────────────────────────
// Векторные операции (sf::Vector2f)
// ─────────────────────────────────────────────────────────────────────────────

// 2D «крест-произведение»: скалярная z-компонента (a × b).
// Результат > 0 — b левее a (CCW), < 0 — правее (CW).
// Используется в PhysicsBody::applyForceAtPoint для вычисления торка.
[[nodiscard]] constexpr float cross2D(sf::Vector2f a, sf::Vector2f b) noexcept {
    return a.x * b.y - a.y * b.x;
}

// Скалярное произведение
[[nodiscard]] constexpr float dot(sf::Vector2f a, sf::Vector2f b) noexcept {
    return a.x * b.x + a.y * b.y;
}

// Длина вектора. std::hypot численно устойчивее, чем sqrt(x²+y²)
[[nodiscard]] inline float length(sf::Vector2f v) noexcept {
    return std::hypot(v.x, v.y);
}

// Длина вектора в квадрате — когда нужно только сравнить, без sqrt
[[nodiscard]] constexpr float lengthSq(sf::Vector2f v) noexcept {
    return v.x * v.x + v.y * v.y;
}

// Нормализация вектора. Возвращает нулевой вектор если длина < eps.
// Не делай normalize без проверки — деление на ~0 это UB в физике симуляции.
[[nodiscard]] inline sf::Vector2f normalize(sf::Vector2f v, float eps = 1e-6f) noexcept {
    const float len = length(v);
    return (len > eps) ? v / len : sf::Vector2f{0.f, 0.f};
}

// Расстояние между двумя точками
[[nodiscard]] inline float distance(sf::Vector2f a, sf::Vector2f b) noexcept {
    return length(b - a);
}

// Линейная интерполяция векторов — для Renderer (сглаживание между тиками)
[[nodiscard]] constexpr sf::Vector2f lerp(sf::Vector2f a, sf::Vector2f b, float t) noexcept {
    return { lerp(a.x, b.x, t), lerp(a.y, b.y, t) };
}

// ─────────────────────────────────────────────────────────────────────────────
// Поворот вектора на угол (радианы)
// Применяется в DockingPort::worldPosition для перевода localOffset в мировые СК
// ─────────────────────────────────────────────────────────────────────────────
[[nodiscard]] inline sf::Vector2f rotate(sf::Vector2f v, float angle) noexcept {
    const float c = std::cos(angle);
    const float s = std::sin(angle);
    return { v.x * c - v.y * s,
             v.x * s + v.y * c };
}