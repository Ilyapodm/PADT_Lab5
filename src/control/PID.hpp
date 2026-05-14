#pragma once

#include <algorithm>   // std::clamp


// Конфиг для Пид регулятора
struct PIDConfig {
    float kp, ki, kd;
    float integralLimit = 50.f;
};

struct PID {
    float kp = 0.f, ki = 0.f, kd = 0.f;
    float integralLimit = 50.f;   

    float prevError    = 0.f;
    float integral     = 0.f;
    
    bool initialized_ = false;  // избавляемся от derivative kick

    [[nodiscard]] float update(float error, float dt) {
	        if (!initialized_) {
		        prevError    = error;
		        initialized_ = true;
		        return 0.f;  // первый тик — только инициализация, никакого вывода    
		    }
		    
        integral += error * dt;
        // Anti-windup: ограничиваем накопленный интеграл
        integral = std::clamp(integral, -integralLimit, integralLimit);

        float deriv = (dt > 1e-6f) ? (error - prevError) / dt : 0.f;
        prevError = error;

        return kp * error + ki * integral + kd * deriv;
    }

    void reset() { prevError = 0.f; integral = 0.f; initialized_ = false; }
};