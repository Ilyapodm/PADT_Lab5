#pragma once

#include <algorithm>   // std::clamp


// Конфиг для Пид регулятора
struct PIDConfig {
    float kp, ki, kd;
    float integral_limit = 50.f;
};

struct PID {
    float kp = 0.f, ki = 0.f, kd = 0.f;
    float integral_limit = 50.f;   

    float prev_error    = 0.f;
    float integral     = 0.f;
    
    bool initialized_ = false;  // избавляемся от derivative kick

    [[nodiscard]] float update(float error, float dt) {
	        if (!initialized_) {
		        prev_error    = error;
		        initialized_ = true;
		        return 0.f;  // первый тик — только инициализация, никакого вывода    
		    }
		    
        integral += error * dt;
        // Anti-windup: ограничиваем накопленный интеграл
        integral = std::clamp(integral, -integral_limit, integral_limit);

        float deriv = (dt > 1e-6f) ? (error - prev_error) / dt : 0.f;
        prev_error = error;

        return kp * error + ki * integral + kd * deriv;
    }

    void reset() { prev_error = 0.f; integral = 0.f; initialized_ = false; }
};

// Создание PID из конфига
[[nodiscard]] inline PID make_PID(const PIDConfig& cfg) noexcept {
    PID pid;
    pid.kp            = cfg.kp;
    pid.ki            = cfg.ki;
    pid.kd            = cfg.kd;
    pid.integral_limit = cfg.integral_limit;
    return pid;
}