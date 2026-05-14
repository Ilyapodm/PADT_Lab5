// C++20

// Конфиг для Автопилота
struct AutopilotConfig {
    // Пороги переходов между фазами
    float alignToApproachAngle   = 4.f;   // градусы
    float approachToAlignAngle   = 8.f;
    float approachToFinalDx      = 50.f;  // метры
    float approachToFinalAngle   = 3.f;
    float finalToApproachDx      = 70.f;
    float finalToApproachAngle   = 6.f;

    // Пороги успешной стыковки
    float dockedDist             = 0.5f;  // метры
    float dockedAngle            = 1.f;   // градусы
    float dockedSpeed            = 0.1f;  // м/с

    // Максимальная скорость при контакте
    float maxDockingSpeed        = 0.5f;  // м/с
};


// конфиг коэффициентов для Пид регуляторов Автопилота
struct AutopilotPIDConfig {
    PIDConfig theta = { 2.0f, 0.02f, 3.0f  };
    PIDConfig x     = { 0.8f, 0.01f, 1.5f  };
    PIDConfig y     = { 0.5f, 0.005f, 1.2f };
    PIDConfig dampX = { 1.2f, 0.05f, 0.8f  };
    PIDConfig dampY = { 1.2f, 0.05f, 0.8f  };
};

// выход автопилота
struct ThrustCommand {
    std::vector<float> throttles; // размер == thruster.count()
};

// FSM состояния
enum class Phase {
    ALIGN,
    APPROACH,
    FINAL,
    DOCKED,
    EXPLODED   // терминальный, без возврата
};

// структура для публичного геттера для рендеринга
struct AutopilotDebugInfo {
    bool pidTheta, pidX, pidY;
    bool dampX, dampY;
};


class Autopilot {
public:
    // Инициализация — привязка к двигателям, однократно
    void init(const Thruster& t);

    // Установка целевой точки стыковки
    void setTarget(sf::Vector2f pos, float angle);

    // Главный метод — вызывается каждый физический тик из Scene
    [[nodiscard]] ThrustCommand update(const PhysicsBody& module,
                                       const DockingTarget& target,
                                       float dt);

    // Уведомления от Scene о внешних событиях
    void notifyDocked();
    void notifyFailed();

    // Геттер фазы — для HUD и Scene
    [[nodiscard]] Phase phase() const;

    // debug для рендеринга
    [[nodiscard]] AutopilotDebugInfo debugInfo() const;  

private:
    // ── Зависимости ─────────────────────────────────────────────────
    const Thruster* thruster_ = nullptr;

    // ── Состояние FSM ───────────────────────────────────────────────
    Phase phase_ = Phase::ALIGN;

    // ── Цель стыковки ───────────────────────────────────────────────
    sf::Vector2f targetPos_;
    float        targetAngle_ = 0.f;

    // ── PID регуляторы (позиция) ────────────────────────────────────
    PID pidTheta_, pidX_, pidY_;

    // ── Velocity dampers (скорость → 0) ────────────────────────────
    PID dampX_, dampY_;

    // ── Индексы двигателей (заполняются в init()) ───────────────────
    ThrusterID idMain_;
    ThrusterID idRcsLeft_,  idRcsRight_;
    ThrusterID idBwdLeft_,  idBwdRight_;
    ThrusterID idRotCwL_,   idRotCwR_;
    ThrusterID idRotCcwL_,  idRotCcwR_;


    // ── FSM ─────────────────────────────────────────────────────────
    // Проверяет условия переходов, переключает phase_,
    // вызывает reset() нужных PID/damper
    void updatePhase(const PhysicsBody& module, const DockingTarget& target);

    // ── Control ─────────────────────────────────────────────────────
    // Считает ThrustCommand для текущей фазы
    [[nodiscard]] ThrustCommand computeThrust(const PhysicsBody& module,
                                              const DockingTarget& target,
                                              float dt);

    // Управление по отдельным осям — вызываются из computeThrust()
    [[nodiscard]] float computeThetaControl(const PhysicsBody& module,
                                            const DockingTarget& target,
                                            float dt);

    [[nodiscard]] float computeXControl(const PhysicsBody& module,
                                        const DockingTarget& target,
                                        float dt);

    [[nodiscard]] float computeYControl(const PhysicsBody& module,
                                        const DockingTarget& target,
                                        float dt);

    // Velocity dampers — гашение остаточных скоростей
    [[nodiscard]] float computeDampX(const PhysicsBody& module, float dt);
    [[nodiscard]] float computeDampY(const PhysicsBody& module, float dt);

    // θ-PID активен во всех управляемых фазах (ALIGN, APPROACH, FINAL)
    [[nodiscard]] bool isPidThetaActive()  const { return true; }

    // X-PID (боковое смещение) — активен когда нужно выходить на ось стыковки
    [[nodiscard]] bool isPidXActive()  const { return phase_ == Phase::APPROACH || phase_ == Phase::FINAL; }

    // Y-PID (продольное сближение) — только в финальном заходе
    [[nodiscard]] bool isPidYActive()  const { return phase_ == Phase::FINAL; }

    // dampX — гасит остаточную скорость по X когда X-PID не управляет осью
	[[nodiscard]] bool isDampXActive() const { return phase_ == Phase::ALIGN; }

    // dampY — гасит остаточную скорость по Y когда Y-PID не управляет осью
    [[nodiscard]] bool isDampYActive() const { return phase_ == Phase::ALIGN    || phase_ == Phase::APPROACH; }
};