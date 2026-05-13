
struct PID {
    float kp, ki, kd;

    float prevError = 0.f;
    float integral  = 0.f;
    
    bool initialized_ = false;  // избавляемся от derivative kick

    [[nodiscard]] float update(float error, float dt) {
	        if (!initialized_) {
		        prevError    = error;
		        initialized_ = true;
		        return 0.f;  // первый тик — только инициализация, никакого вывода    
		    }
		    
        integral += error * dt;
        // Anti-windup: ограничиваем накопленный интеграл
        integral = std::clamp(integral, -50.f, 50.f);

        float deriv = (dt > 1e-6f) ? (error - prevError) / dt : 0.f;
        prevError = error;

        return kp * error + ki * integral + kd * deriv;
    }

    void reset() { prevError = 0.f; integral = 0.f; initialized_ = false; }
};