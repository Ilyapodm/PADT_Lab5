#include "physics_body.hpp"
#include <cassert>

// set_mass: меняет массу И пересчитывает момент инерции
void PhysicsBody::set_mass(float m) {
    assert(m > 0.f);   // масса не может быть нулевой или отрицательной
    mass_    = m;
    // I = m(w² + h²) / 12  — для прямоугольного тела
    inertia_ = mass_ * (width * width + height * height) / 12.f;
}

// set_size: меняет размеры И пересчитывает inertia
void PhysicsBody::set_size(float w, float h) {
    assert(w > 0.f && h > 0.f);
    width  = w;
    height = h;
    inertia_ = mass_ * (width * width + height * height) / 12.f;
}