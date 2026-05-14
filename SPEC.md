# Docking Simulation — Архитектура и важные моменты

> **Стек:** C++20 · SFML 2.6+ · CMake + Ninja · Clang++  
> **Задача:** Симуляция автоматической стыковки космического модуля со станцией в условиях гравитации и атмосферного ветра.

---

## Структура проекта

```
docking-sim/
├── CMakeLists.txt
├──src/
│   ├── simulation/
│	│	├── scene.hpp / .cpp             // владеет всеми объектами симуляции
│   │   ├── physics_body.hpp / .cpp      // масса, скорость, силы, момент инерции
│   │   ├── atmosphere.hpp / .cpp        // ветер, аэродинамическое сопротивление
│   │   ├── gravity.hpp / .cpp           // гравитационное поле
│   │   ├── thruster.hpp / .cpp          // модель двигателей (RCS + main)
│   │   └── docking_port.hpp / .cpp      // геометрия стыковки, детекция
│   ├── control/
│   │   ├── PID.hpp                      // универсальный PID-регулятор
│   │   └── autopilot.hpp / .cpp         // фазовый автомат + управление тягой
│   ├── rendering/
│   │   └── renderer.hpp / .cpp          // SFML-слой, масштаб, HUD
│   └── utils/
│       └── math_utils.hpp               // clamp, lerp, cross2D, нормализация угла
```

---

## Слой 1 — Физическая модель (`physics_body`)

### Что хранит объект

```cpp
// C++20
struct PhysicsBody {
    sf::Vector2f position, velocity;
    float angle = 0.f, angularVel = 0.f;
    sf::Vector2f netForce;
    float netTorque = 0.f;

    [[nodiscard]] float mass()    const { return mass_; }
    [[nodiscard]] float inertia() const { return inertia_; }
    void setMass(float m);
    void setInertia(float i);
    // ...
private:
    float mass_    = 1.f;
    float inertia_ = 1.f;
};

```

### Интегратор — Symplectic Euler

Обычный Euler неустойчив при упругих силах. Symplectic (полунеявный) Euler устойчив и дёшев:

```cpp
void PhysicsBody::integrate(float dt) {
    // 1. Сначала обновляем скорости
    velocity   += (netForce  / mass_)    * dt;
    angularVel += (netTorque / inertia_) * dt;
    // 2. Затем позиции (используют уже новые скорости)
    position   += velocity    * dt;
    angle      += angularVel  * dt;
}
```

### Торк от силы в точке

Сила, приложенная не в центре масс, создаёт и линейное ускорение, и вращение:

```cpp
void PhysicsBody::applyForceAtPoint(sf::Vector2f f, sf::Vector2f offset) {
    netForce  += f;
    netTorque += offset.x * f.y - offset.y * f.x;  // 2D cross product
}
```

---

## Слой 2 — Окружение

### Гравитация

```cpp
class Gravity {
public:
    explicit Gravity(float g = 9.81f) : g_{g} {}

    [[nodiscard]] sf::Vector2f forceOn(const PhysicsBody& body) const {
        return { 0.f, body.mass() * g_ };  // +Y = вниз (экранные координаты SFML)
    }
private:
    float g_;
};
```

### Атмосфера и ветер

Сила аэродинамического сопротивления:

$$F_{drag} = \frac{1}{2} \rho C_d A \, |v_{rel}|^2 \cdot \hat{v}_{rel}$$

где $$(v_{rel} = v_{body} - v_{wind})$$ — скорость тела относительно воздушной массы.

```cpp
class Atmosphere {
public:
    // Синусоидальный порывистый ветер + шум
    [[nodiscard]] sf::Vector2f windAt(float time) const;
    [[nodiscard]] sf::Vector2f dragForce(const PhysicsBody& body, float time) const {
		// В начало dragForce():
		sf::Vector2f relVel = body.velocity - windAt(time);
		float speed = std::hypot(relVel.x, relVel.y);  // hypot безопаснее sqrt
		if (speed < 1e-4f) return {};  // нет относительного движения — нет силы
		
		sf::Vector2f dir = relVel / speed;  // нормализация только после проверки
		return -0.5f * airDensity_ * dragCoeff_ * frontArea_ * speed * speed * dir;    
    }

private:
    sf::Vector2f baseWind_     = { 5.f, 0.f };  // м/с, постоянная составляющая
    float        gustAmplitude_ = 3.f;           // м/с, амплитуда порывов
    float        gustFrequency_ = 0.3f;          // Гц
    float        airDensity_    = 1.2f;          // кг/м³
    float        dragCoeff_     = 0.47f;         // Cd сферы ≈ 0.47
    float        frontArea_     = 4.f;           // м², фронтальная площадь
};
```

> **Важно:** ветер входит в формулу через **относительную** скорость.  
> При встречном ветре сопротивление может стать ускоряющей силой — это физически корректно.

---

## Слой 3 — Двигатели (`Thruster`)

Модуль имеет **8 RCS-движков** (маневровые) + **1 главный**.  
Каждый движок расположен в конкретной точке корпуса и имеет направление тяги в локальной СК.

| Label         | localOffset  | localDir | Назначение              |
| ------------- | ------------ | -------- | ----------------------- |
| main_fwd      | (0, +H/2)    | (0, −1)  | Сближение со станцией   |
| rcs_bwd_left  | (−W/2, +H/4) | (0, +1)  | Торможение (левый)      |
| rcs_bwd_right | (+W/2, +H/4) | (0, +1)  | Торможение (правый)     |
| rcs_left      | (−W/2, 0)    | (−1, 0)  | Движение влево          |
| rcs_right     | (+W/2, 0)    | (+1, 0)  | Движение вправо         |
| rcs_rot_cw_l  | (−W/2, −H/4) | (0, −1)  | Вращение по часовой     |
| rcs_rot_cw_r  | (+W/2, +H/4) | (0, +1)  | Вращение по часовой     |
| rcs_rot_ccw_l | (−W/2, +H/4) | (0, +1)  | Вращение против часовой |
| rcs_rot_ccw_r | (+W/2, −H/4) | (0, −1)  | Вращение против часовой |
Координаты: Y - смотрит вниз, X - направо
```
                           [ СТАНЦИЯ ]
                                ↑
══════════════════════════════════════════════════════════════════════════════

                 Нос, место стыковки (y = -H/2)
               ┌─────────────────────────────────┐
               │                                 │
               │[rcs_rot_cw_l]    [rcs_rot_ccw_r]│
               ⬮(−W/2, −H/4)         (+W/2, −H/4)⬮
               │     →→→                   ←←←   │
               │                                 │
               │                                 │
               │                                 │
               │                                 │
               │                                 │
               │                                 │
               │          центр масс             │
    [rcs_left] ⬮             [ + ]               ⬮ [rcs_right]
    (-W/2, 0)  │                                 │  (+W/2, 0)
      ←←←      │                                 │    →→→
               │                                 │
               │                                 │
               │                                 │
[rcs_bwd_left] │                                 │ [rcs_bwd_right]
 (-W/2,+H/4)   ⬮[rcs_rot_ccw_l]    [rcs_rot_cw_r]⬮    (+W/2,+H/4)
    ↓ ↓ ↓      │(-W/2, +H/4)         (+W/2, +H/4)│      ↓ ↓ ↓
               │   →→→                    ←←←    │
               │                                 │
               │               ↑↑↑               │
               └────────────────⬮────────────────┘ 
                         [main_fwd](0, +H/2)
                            Хвост
```

При чистом торможении - оба тормозных двигателя (rcs_bwd_left, rcs_bwd_right) включаются симметрично с одинаковым throttle


```cpp
struct ThrusterConfig {
    sf::Vector2f localOffset;  // от центра масс, м
    sf::Vector2f localDir;     // единичный вектор направления тяги
    float        maxForce;     // Н
    std::string label;          // "main_fwd", "rcs_left", etc. — для HUD
};

enum class ThrusterID : std::size_t {};  // строго типизированный индекс

class Thruster {
    std::vector<ThrusterConfig> configs_;
    std::unordered_map<std::string, ThrusterID> labels_;
public:
    explicit Thruster(std::vector<ThrusterConfig> configs);

    // Автопилот получает индекс по имени — один раз при инициализации:
    [[nodiscard]] ThrusterID id(std::string_view label) const;

    // Команда — массив нормированных тяг, индексированный через ThrusterID:
    // throttles - как сильно давить от PID регулятора
    void applyCommands(PhysicsBody&, std::span<const float> throttles) const;
    [[nodiscard]] std::size_t count() const { return configs_.size(); }
};
```

Тяга каждого движка применяется через `applyForceAtPoint` — автоматически генерируя нужный торк.

---

## Слой 4 — Автопилот

#### PID-регулятор

```cpp
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
[[nodiscard]] inline PID make_PID(const PIDConfig& cfg) noexcept;
```

**Anti-windup обязателен:** без него при длительном рассогласовании `integral` накопится до огромных значений и автопилот "взорвётся" при достижении цели.

### Фазовый автомат (`Autopilot`)

Три степени свободы (X, Y, θ) управляются **последовательно по фазам**:

Таблица переходов

| Откуда   | Куда     | Условие входа                                  | Активация | Отключаем + сбрасываем состояние |
| -------- | -------- | ---------------------------------------------- | --------- | -------------------------------- |
| ALIGN    | APPROACH | \|dθ\| < 4°                                    | X-PID     | -                                |
| APPROACH | ALIGN    | \|dθ\| > 8°                                    | Vx-damper | X-PID                            |
| APPROACH | FINAL    | \|dX\| < 50 м && \|dθ\| < 3°                   | Y-PID     | Vy-damper                        |
| FINAL    | APPROACH | \|dX\| > 70 м \|\| \|dθ\| > 6°                 | Vy-damper | Y-PID                            |
| FINAL    | DOCKED   | dist < 0.5 м && \|dθ\| < 1° && \|v\| < 0.1 м/с |           |                                  |
| FINAL    | EXPLODED | dist < 0.5 м && \|v\| > v_max                  |           |                                  |
| APPROACH | EXPLODED | dist < 0.5 м && \|v\| > v_max                  |           |                                  |
Сброс позиционного PID происходит **при выходе** из фазы, где он был активен, а не при входе в следующую.


Какие PID активны в каждой фазе

| Фаза     | θ-PID | X-PID | Y-PID | Vx-damper | Vy-damper | Логика                                        |
| -------- | ----- | ----- | ----- | --------- | --------- | --------------------------------------------- |
| ALIGN    | ✅     | ❌     | ❌     | ✅         | ✅         | Только поворот + гашение остаточных скоростей |
| APPROACH | ✅     | ✅     | ❌     | ❌         | ✅         | Выход на ось + удержание угла                 |
| FINAL    | ✅     | ✅     | ✅     | ❌         | ❌         | Все три оси, медленно                         |
| DOCKED   | ❌     | ❌     | ❌     | ❌         | ❌         | Двигатели выключены                           |
| EXPLODED | ❌     | ❌     | ❌     | ❌         | ❌         | Всё выключено                                 |


```cpp
// Конфиг для Автопилота
struct AutopilotConfig {
    // Пороги переходов между фазами
    float align_to_approach_angle   = 4.f;   // градусы
    float approach_to_align_angle   = 8.f;
    float approach_to_final_dx      = 50.f;  // метры
    float approach_to_final_angle   = 3.f;
    float final_to_approach_dx      = 70.f;
    float final_to_approach_angle   = 6.f;

    // Пороги успешной стыковки
    float docked_dist             = 0.5f;  // метры
    float docked_angle            = 1.f;   // градусы
    float docked_speed            = 0.1f;  // м/с

    // Максимальная скорость при контакте
    float maxDocking_speed        = 0.5f;  // м/с
};


// конфиг коэффициентов для Пид регуляторов Автопилота
struct AutopilotPIDConfig {
    PIDConfig theta = { 2.0f, 0.02f, 3.0f  };
    PIDConfig x     = { 0.8f, 0.01f, 1.5f  };
    PIDConfig y     = { 0.5f, 0.005f, 1.2f };
    PIDConfig damp_x = { 1.2f, 0.05f, 0.8f  };
    PIDConfig damp_y = { 1.2f, 0.05f, 0.8f  };
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
    bool pid_theta, pid_x, pid_y;
    bool damp_x, damp_y;
};


class Autopilot {
public:
    // Инициализация — привязка к двигателям, однократно
    void init(const Thruster& t);

    // Установка целевой точки стыковки
    void set_target(sf::Vector2f pos, float angle);

    // Главный метод — вызывается каждый физический тик из Scene
    [[nodiscard]] ThrustCommand update(const PhysicsBody& module,
                                       const DockingTarget& target,
                                       float dt);

    // Уведомления от Scene о внешних событиях
    void notify_docked();
    void notify_failed();

    // Геттер фазы — для HUD и Scene
    [[nodiscard]] Phase phase() const;

    // debug для рендеринга
    [[nodiscard]] AutopilotDebugInfo debug_info() const;  

private:
    // ── Зависимости ─────────────────────────────────────────────────
    const Thruster* thruster_ = nullptr;

    // ── Состояние FSM ───────────────────────────────────────────────
    Phase phase_ = Phase::ALIGN;

    // ── Цель стыковки ───────────────────────────────────────────────
    sf::Vector2f target_pos_;
    float        target_angle_ = 0.f;

    // ── PID регуляторы (позиция) ────────────────────────────────────
    PID pid_theta_, pid_x_, pid_y_;

    // ── Velocity dampers (скорость → 0) ────────────────────────────
    PID damp_x_, damp_y_;

    // ── Индексы двигателей (заполняются в init()) ───────────────────
    ThrusterID id_main_;
    ThrusterID id_rcs_left_,  id_rcs_right_;
    ThrusterID id_bwd_left_,  id_bwd_right_;
    ThrusterID id_rot_cwl_,   id_rot_cwr_;
    ThrusterID id_rot_ccwl_,  id_rot_ccwr_;


    // ── FSM ─────────────────────────────────────────────────────────
    // Проверяет условия переходов, переключает phase_,
    // вызывает reset() нужных PID/damper
    void update_phase(const PhysicsBody& module, const DockingTarget& target);

    // ── Control ─────────────────────────────────────────────────────
    // Считает ThrustCommand для текущей фазы
    [[nodiscard]] ThrustCommand compute_thrust(const PhysicsBody& module,
                                              const DockingTarget& target,
                                              float dt);

    // Управление по отдельным осям — вызываются из computeThrust()
    [[nodiscard]] float compute_theta_control(const PhysicsBody& module,
                                            const DockingTarget& target,
                                            float dt);

    [[nodiscard]] float compute_x_control(const PhysicsBody& module,
                                        const DockingTarget& target,
                                        float dt);

    [[nodiscard]] float compute_y_control(const PhysicsBody& module,
                                        const DockingTarget& target,
                                        float dt);

    // Velocity dampers — гашение остаточных скоростей
    [[nodiscard]] float compute_damp_x(const PhysicsBody& module, float dt);
    [[nodiscard]] float compute_damp_y(const PhysicsBody& module, float dt);

    // θ-PID активен во всех управляемых фазах (ALIGN, APPROACH, FINAL)
    [[nodiscard]] bool isPid_theta_active()  const { return true; }

    // X-PID (боковое смещение) — активен когда нужно выходить на ось стыковки
    [[nodiscard]] bool isPid_x_active()  const { return phase_ == Phase::APPROACH || phase_ == Phase::FINAL; }

    // Y-PID (продольное сближение) — только в финальном заходе
    [[nodiscard]] bool isPid_y_active()  const { return phase_ == Phase::FINAL; }

    // dampX — гасит остаточную скорость по X когда X-PID не управляет осью
	[[nodiscard]] bool is_damp_x_active() const { return phase_ == Phase::ALIGN; }

    // dampY — гасит остаточную скорость по Y когда Y-PID не управляет осью
    [[nodiscard]] bool is_damp_y_active() const { return phase_ == Phase::ALIGN || phase_ == Phase::APPROACH; }
};
```

Так `compute_thrust` — единственное место, где решается «кого вызвать», а `pid.update()` скрыт внутри каждого `compute*`

### Начальные значения PID (отправная точка для тюнинга)

| Ось | kp | ki | kd |
|---|---|---|---|
| X (поперечное) | 0.8 | 0.01 | 1.5 |
| Y (продольное) | 0.5 | 0.005 | 1.2 |
| θ (угол) | 2.0 | 0.02 | 3.0 |

> **Алгоритм тюнинга:** сначала ki = kd = 0, увеличивай kp до появления осцилляций,  
> затем kd для их гашения, и лишь потом ki для устранения статической ошибки.

---

## Слой 5 — Стыковочная геометрия

```cpp
struct DockingPort {
    sf::Vector2f localOffset;   // смещение от центра масс тела, м
    float        localAngle;    // угол порта в локальной СК тела, рад

    // Позиция и угол в мировых координатах
    [[nodiscard]] sf::Vector2f worldPosition(const PhysicsBody& body) const;
    [[nodiscard]] float        worldAngle   (const PhysicsBody& body) const;
};

struct DockingResult { bool success; float speed; float angleDiff; };

[[nodiscard]] DockingResult checkDocking(
    const DockingPort& portA, const PhysicsBody& bodyA,
    const DockingPort& portB, const PhysicsBody& bodyB,
    float posThreshold   = 0.5f,    // м
    float angleThreshold = 0.0175f, // рад ≈ 1°
    float speedThreshold = 0.1f     // м/с
);

struct DockingTarget {
    sf::Vector2f approachPoint;  // точка за портом, откуда начинать заход
    sf::Vector2f portPosition;   // мировые координаты порта станции
    float        portAngle;      // требуемый угол подхода
};

// Вычисление — в DockingPort:
[[nodiscard]] DockingTarget computeTarget(
    const DockingPort& stationPort,
    const PhysicsBody& station) const;
```

---

## Слой 6 — Game Loop (main.cpp)

### Фиксированный физический шаг + accumulator

```cpp
// C++20
#include "simulation/Scene.hpp"
#include "rendering/Renderer.hpp"
#include <SFML/Graphics.hpp>
#include <algorithm>  // std::min

int main() {
    sf::RenderWindow window({1280u, 720u}, "Docking Sim");
    window.setFramerateLimit(60);

    constexpr float PHYSICS_DT = 1.f / 120.f;  // 120 Hz физика
    float accumulator = 0.f;
    sf::Clock clock;

    Scene    scene;    // владеет всей симуляцией
    Renderer renderer; // владеет только отображением

    while (window.isOpen()) {
        // --- Обработка событий ---
        for (sf::Event event; window.pollEvent(event);) {
            if (event.type == sf::Event::Closed)
                window.close();
            if (event.type == sf::Event::KeyPressed &&
                event.key.code == sf::Keyboard::R)
                scene.reset();  // R — перезапуск симуляции
        }

        // --- Accumulator: физика независима от FPS ---
        float frameTime = clock.restart().asSeconds();
        frameTime = std::min(frameTime, 0.05f);  // spiral of death protection
        accumulator += frameTime;

        while (accumulator >= PHYSICS_DT) {
            scene.update(PHYSICS_DT);   // один физический тик
            accumulator -= PHYSICS_DT;
        }

        // --- Рендеринг ---
        window.clear(sf::Color{8, 8, 20});
        renderer.draw(window, scene);
        window.display();
    }
}
```

> `std::min(frameTime, 0.05f)` — "spiral of death" protection:  
> если кадр занял > 50 мс (лаг), не пытаемся наверстать физику бесконечным циклом.

```cpp
// C++20
class Scene {
public:
    explicit Scene(/* config */);
    void update(float dt);  // один физический тик
    void reset();           // перезапуск симуляции
    [[nodiscard]] const PhysicsBody& moduleBody()  const;
    [[nodiscard]] Autopilot::Phase   phase()        const;
    // ...
private:
    PhysicsBody  module_, station_;
    Gravity      gravity_;
    Atmosphere   atmosphere_;
    Thruster     thruster_;
    Autopilot    autopilot_;
    DockingPort  modulePort_, stationPort_;
    float totalTime_ = 0.f;
    bool stationDocked_ = false;
    void mergeBodies();
};

// C++20 — Scene.cpp
void Scene::update(float dt) {
	// ── МОДУЛЬ ──────────────────────────────────────────
    // 1. Внешние силы
    module_.applyForce(gravity_.forceOn(module_));
    module_.applyForce(atmosphere_.dragForce(module_, totalTime_));

    // 2. Автопилот → управляющие силы
    DockingTarget target = computeDockingTarget(
        modulePort_, module_, stationPort_, station_);
    auto cmd = autopilot_.update(module_, target, dt);
	thruster_.applyCommands(module_, cmd.throttles);

    // 3. Интегрирование + сброс
    module_.integrate(dt);
    module_.resetForces();
    
    // ── СТАНЦИЯ ─────────────────────────────────────────────────────
    // Станция — полноценное PhysicsBody: движется под действием
    // гравитации и атмосферного сопротивления (ветра).
    // Стыковка не является мгновенной «заморозкой» — станция продолжает
    // двигаться до фактического объединения тел.
    // DockingTarget пересчитывается КАЖДЫЙ тик (см. выше),
    // поэтому автопилот всегда нацелен на актуальную позицию порта
    if (!stationDocked_) {        
	    station_.applyForce(gravity_.forceOn(station_));        
	    station_.applyForce(atmosphere_.dragForce(station_, totalTime_));     
	    
	    station_.integrate(dt);
	    station_.resetForces();   
	}

    // ── Стыковка ───────────────────────────────────────── 
    auto result = checkDocking(modulePort_, module_, stationPort_, station_);
    if (result.success) {
	    autopilot_.notifyDocked();
	    mergeBodies(); // // ← пересчёт CM, массы, инерции, скоростей
	} else {
	    const bool contact = /* dist(portA, portB) < posThreshold */;
	    if (contact && result.speed > maxDockingSpeed_)
	        autopilot_.notifyFailed();
	}

    totalTime_ += dt;
}
```

---

## Слой 7 — Рендеринг (`Renderer`)

### Масштаб

```cpp
class Renderer {
public:
    explicit Renderer(float pixelsPerMeter = 10.f) 
        : pixelsPerMeter_{pixelsPerMeter} {}

    void setScale(float ppm) { pixelsPerMeter_ = ppm; }

private:
    float        pixelsPerMeter_;
    sf::Vector2f cameraOffset_ = {};  // для будущей прокрутки/следования
};

// Физика — в метрах. Renderer — единственное место, где применяется масштаб.
sf::Vector2f toScreen(sf::Vector2f worldPos) {
    return worldPos * pixelsPerMeter_ + cameraOffset_;
    // + при необходимости смещение камеры
}

// ДОБАВИТЬ — структура для хранения двух состояний (нужна при реализации интерполяции):
// struct BodySnapshot { sf::Vector2f position; float angle; };
//
// В PhysicsBody добавить:
// BodySnapshot snapshot() const { return { position, angle }; }
//
// В game loop, перед integrate():
// prevSnapshot = module.snapshot();
//
// В Renderer::draw():
// float alpha = accumulator / PHYSICS_DT;
// sf::Vector2f renderPos = prevSnapshot.position * (1.f - alpha) + module.position * alpha;
// float renderAngle      = std::lerp(prevSnapshot.angle, module.angle, alpha);
//
// ТЕХНИЧЕСКИЙ ДОЛГ: при переходе к 144+ FPS — раскомментировать и активировать.
```

### HUD — что отображать

- Фаза автопилота (ALIGN / APPROACH / FINAL / DOCKED / EXPLODED)
- Расстояние до порта (м)
- Скорость сближения (м/с)
- Угловое рассогласование (°)
- Вектор ветра (стрелка)
- Тяга каждого движка (цветные индикаторы)

---

## Критические архитектурные моменты

### 1. Система координат

SFML использует **Y↓** (ось Y направлена вниз). Все физические расчёты ведутся в этой же СК.  
Гравитация: `{0.f, +g}`. Документируй это явно в каждом файле, работающем с углами и позициями.

### 2. Нормализация углов

При вычислении угловой ошибки для PID обязательно нормализуй в `[-π, π]`:

```cpp
// O(1), без цикла:
[[nodiscard]] constexpr float normalizeAngle(float a) noexcept {
    return std::remainder(a, 2.f * std::numbers::pi_v<float>);
}
```

Без этого PID "видит" ошибку 350° вместо -10° и разворачивает модуль не в ту сторону.

### 3. Момент инерции

Не подбирай `inertia` произвольно. Для прямоугольного тела:

$$I = \frac{m (w^2 + h^2)}{12}$$

Неверный `inertia` делает ротацию либо "дубовой", либо "стеклянной" — оба варианта разрушают тюнинг PID.

### 4. Ветер и PID

Постоянный ветер — это постоянное возмущение. PID с `ki > 0` (интегральная составляющая) обязательно его скомпенсирует. Без `ki` автопилот будет удерживать позицию со статической ошибкой.

### 5. Порядок операций за тик

```
1. Накопить внешние силы (gravity, atmosphere)
2. Накопить силы управления (thruster)
3. integrate(dt)          → обновить скорости и позиции
4. resetForces()          → сбросить netForce/netTorque
5. Проверить докинг
6. Рендерить
```

Нарушение порядка (например, resetForces до integrate) — физически неверное поведение.

### 6. `[[nodiscard]]` везде, где важен результат

`checkDocking`, `windAt`, `worldPosition` — все чистые функции должны быть `[[nodiscard]]`.  
Компилятор предупредит, если результат случайно проигнорирован.

### 7. Residual velocity при откате фазы.
При переходе на более низкую фазу (FINAL → APPROACH → ALIGN) скорость по осям, которые выходят из управления позиционного PID, не обнуляется физически. Для каждой такой оси активировать velocity damper (PID с setpoint = 0) с немедленным включением в момент перехода и ki > 0 для компенсации гравитационного дрейфа.

### 8. Объединение тел после стыковки (Compound Body)

После `checkDocking` → `success`: вызвать `Scene::mergeBodies()`.
Пересчитать: массу (сумма), центр масс (взвешенное среднее),
скорость (закон импульса), момент инерции (теорема Штейнера),
угловую скорость (закон момента импульса).
Установить `stationDocked_ = true` — модуль исключается из
физического цикла. Compound-тело продолжает движение под
действием гравитации и ветра как единый объект.


---

## Порядок реализации (рекомендуемый)

1. **PhysicsBody + интегратор** — проверить равномерное движение и свободное падение
2. **Gravity** — тест: тело должно ускоряться вниз с 9.81 м/с²
3. **Renderer** — простой прямоугольник-модуль и точка-станция
4. **Ручное управление (WASD)** — почувствовать физику до автопилота
5. **Atmosphere** — добавить ветер и сопротивление, наблюдать снос
6. **Thruster + DockingPort** — модель двигателей, геометрия портов
7. **PID для одной оси** — сначала только Y, отладка kp/kd/ki
8. **Полный Autopilot + фазовый автомат** — три степени свободы
9. **Детекция стыковки + состояния DOCKED/FAILED**
10. **HUD + полировка** — визуальная обратная связь

---

## Сборка (CMakeLists.txt — минимальный шаблон)

```cmake
cmake_minimum_required(VERSION 3.20)
project(docking_sim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(SFML 2.6 COMPONENTS graphics window system REQUIRED)

add_executable(docking_sim
    src/main.cpp
    src/simulation/Scene.cpp
    src/simulation/PhysicsBody.cpp
    src/simulation/Gravity.cpp
    src/simulation/Atmosphere.cpp
    src/simulation/Thruster.cpp
    src/simulation/DockingPort.cpp
    src/control/PID.cpp
    src/control/Autopilot.cpp
    src/rendering/Renderer.cpp
)

target_link_libraries(docking_sim PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_options(docking_sim PRIVATE -Wall -Wextra -Wpedantic)
```