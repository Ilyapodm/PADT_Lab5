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

## Статус реализации

| Файл | Статус | Источник истины |
|---|---|---|
| PID.hpp | В работе | Файл |
| autopilot.hpp / .cpp | В работе | Файл |
| docking_port.hpp / .cpp | Готов | Файл |
| physics_body.hpp / .cpp | Готов | Файл |

Остальное не начато!

> Если файл существует — он авторитетнее SPEC.
> SPEC описывает только то, что ещё не реализовано.

---

## Слой 1 — Физическая модель (`physics_body`)

### Что хранит объект

```cpp
struct PhysicsBody {
    // кинематика
    sf::Vector2f position, velocity;  // мировые координаты, скорость центра масс, м/с
    float angle = 0.f, angular_vel = 0.f;  // угол поворота, угловая скорость

    sf::Vector2f net_force; // накопители сил, сбрасываются каждый тик
    float net_torque = 0.f;

    // геттеры и сеттеры
    [[nodiscard]] float mass()    const { return mass_; }
    [[nodiscard]] float inertia() const { return inertia_; }
    void set_mass(float m);  // пересчитывает inertia
    void set_size(float w, float h);  // пересчитывает inertia
    
    // Геометрия корпуса 
    float width  = 10.f;  // м, по локальной оси X
    float height = 20.f;  // м, по локальной оси Y

    // применение сил через центр масс
    void apply_force(sf::Vector2f f) noexcept;

    // применение сил в произвольную точку
    void apply_force_at_point(sf::Vector2f f, sf::Vector2f offset) noexcept;

    // Интегратор — Symplectic Euler
    void integrate(float dt) noexcept;

    // Сброс накопителей — вызывать ПОСЛЕ integrate()
    void reset_forces() noexcept;

    // Скорость
    [[nodiscard]] float speed() const noexcept;

    // Полудиагональ корпуса — грубый радиус для быстрой broad-phase
    [[nodiscard]] float bounding_radius() const noexcept

    // Снапшот для интерполяции рендеринга 
    struct Snapshot {
        sf::Vector2f position;
        float        angle;
    };
    [[nodiscard]] Snapshot snapshot() const noexcept {
        return { position, angle };
    }

private:
    float mass_    = 1.f;
    float inertia_ = 1.f;
};
```

Интегратор - Symplectic Euler. Обычный Euler неустойчив при упругих силах. Symplectic (полунеявный) Euler устойчив и дёшев

### Что является физической моделью
Модуль и станция - физический объект, на них действует гравитация и ветер, могут быть расположены произвольно в пространстве

---

## Слой 2 — Окружение

### Гравитация

```cpp
// Система координат: Y↓, X→ (SFML). Земля — в начале мировых координат {0, 0}.
// Орбитальная гравитация: сила направлена к earth_center_, убывает как 1/r².
class Gravity {
public:
    // mu = G * M — стандартный гравитационный параметр.
    // Задаётся как mu = v_orb² * r, где v_orb и r — желаемые параметры орбиты.
    // Не используй реальные G и M: радиус орбиты МКС не влезет ни в одно окно.
    explicit Gravity(sf::Vector2f earth_center = {0.f, 0.f},
                     float mu                  = 2'880'000.f)
        : earth_center_{earth_center}
        , mu_{mu}
    {}

    // Gravity::force_on → возвращает вектор к earth_center_ с величиной mu*m/r²
    // Направление: всегда к Земле (не константное «вниз»)
    [[nodiscard]] sf::Vector2f force_on(const PhysicsBody& body) const noexcept;

    [[nodiscard]] float mu() const noexcept { return mu_; }

private:
    sf::Vector2f earth_center_;
    float        mu_;   // G*M, м³/с²
};

// Параметры орбиты. Задай r и v_orb — mu вычислится автоматически.
// Формула: mu = v_orb² * r  (условие круговой орбиты)
struct OrbitConfig {
    float orbit_radius  = 800.f;   // м — расстояние от Земли до станции
    float orbital_speed = 60.f;    // м/с — скорость станции по орбите

    [[nodiscard]] float mu() const noexcept {
        return orbital_speed * orbital_speed * orbit_radius;
    }
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
    [[nodiscard]] sf::Vector2f wind_at(float time) const;
    [[nodiscard]] sf::Vector2f drag_force(const PhysicsBody& body, float time) const;

private:
    sf::Vector2f base_wind_      = { 5.f, 0.f };  // м/с, постоянная составляющая
    float        gust_amplitude_ = 3.f;           // м/с, амплитуда порывов
    float        gust_frequency_ = 0.3f;          // Гц
    float        air_density_    = 1.2f;          // кг/м³
    float        drag_coeff_     = 0.47f;         // Cd сферы ≈ 0.47
    float        front_area_     = 4.f;           // м², фронтальная площадь
};
```

> **Важно:** ветер входит в формулу через **относительную** скорость.  
> При встречном ветре сопротивление может стать ускоряющей силой — это физически корректно.

---

## Слой 3 — Двигатели (`Thruster`)

Модуль имеет 9 двигателей: **8 RCS-движков** (маневровые) + **1 главный**.  
Каждый движок расположен в конкретной точке корпуса и имеет направление тяги в локальной СК.

| Label         | localOffset  | localDir | Назначение              | PID сигнал            |
| ------------- | ------------ | -------- | ----------------------- | --------------------- |
| main_fwd      | (0, +H/2)    | (0, −1)  | Сближение со станцией   | Y-PID > 0, damp_y > 0 |
| rcs_bwd_left  | (−W/2, +H/4) | (0, +1)  | Торможение (левый)      | Y-PID < 0, damp_y < 0 |
| rcs_bwd_right | (+W/2, +H/4) | (0, +1)  | Торможение (правый)     | Y-PID < 0, damp_y < 0 |
| rcs_left      | (−W/2, 0)    | (−1, 0)  | Движение влево          | X-PID < 0, damp_x < 0 |
| rcs_right     | (+W/2, 0)    | (+1, 0)  | Движение вправо         | X-PID > 0, damp_x > 0 |
| rcs_rot_cw_l  | (−W/2, −H/4) | (0, −1)  | Вращение по часовой     | θ-PID > 0             |
| rcs_rot_cw_r  | (+W/2, +H/4) | (0, +1)  | Вращение по часовой     | θ-PID > 0             |
| rcs_rot_ccw_l | (−W/2, +H/4) | (0, +1)  | Вращение против часовой | θ-PID < 0             |
| rcs_rot_ccw_r | (+W/2, −H/4) | (0, −1)  | Вращение против часовой | θ-PID < 0             |
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
    sf::Vector2f local_offset;  // от центра масс, м
    sf::Vector2f local_dir;     // единичный вектор направления тяги
    float        max_force;     // Н
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
    void apply_commands(PhysicsBody&, std::span<const float> throttles) const;
    [[nodiscard]] std::size_t count() const { return configs_.size(); }
};
```

Тяга каждого движка применяется через `apply_force_at_point` — автоматически генерируя нужный торк.

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

    [[nodiscard]] float update(float error, float dt);

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


Какие PID активны в каждой фазе:

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
    float max_docking_speed        = 0.5f;  // м/с
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
    void init(const Thruster& t, const AutopilotPIDConfig& pid_cfg, AutopilotConfig ap_cfg);

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
    [[nodiscard]] bool isPid_theta_active()  const { return phase_ != Phase::DOCKED && phase_ != Phase::EXPLODED; }

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
    // Локальная система координат
    sf::Vector2f local_offset;   // смещение от центра масс тела, м
    float        local_angle;    // угол порта в локальной СК тела, рад

    // Позиция и угол в мировых координатах
    [[nodiscard]] sf::Vector2f world_position(const PhysicsBody& body) const;
    [[nodiscard]] float        world_angle   (const PhysicsBody& body) const;
};

struct DockingResult { bool success; float speed; float angle_diff; };

[[nodiscard]] DockingResult check_docking(
    const DockingPort& port_a, const PhysicsBody& body_a,
    const DockingPort& port_b, const PhysicsBody& body_b,
    float pos_threshold   = 0.5f,    // м
    float angle_threshold = 0.0175f, // рад ≈ 1°
    float speed_threshold = 0.1f     // м/с
);

struct DockingTarget {
    sf::Vector2f approach_point;  // точка за портом, откуда начинать заход
    sf::Vector2f port_position;   // мировые координаты порта станции
    float        port_angle;      // требуемый угол подхода (включает в себя Pi)
};

// Вычисление — в scene.update():
[[nodiscard]] DockingTarget compute_target(
    const DockingPort& station_port,
    const PhysicsBody& station) const;
```

---

## Слой 6 — Game Loop 

### Фиксированный физический шаг + accumulator

```cpp
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
        float frame_time = clock.restart().asSeconds();
        frame_time = std::min(frame_time, 0.05f);  // spiral of death protection
        accumulator += frame_time;

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

> `std::min(frame_time, 0.05f)` — "spiral of death" protection:  
> если кадр занял > 50 мс (лаг), не пытаемся наверстать физику бесконечным циклом.

### Scene
Scene должна в методе update (1 физический тик) должна:
- Применять воздействие физики на тела (двигатели, атмосфера, гравитация)
- Проверять коллизию (сначала через bounding_radius - грубо, потом через SAT (проверка OBB))
- Рассчитывать Snapshot для красивого рендеринга
- Проверять стыковку.
  - Если стыковка удалась, соединить тела (они продолжают двигаться вместе как одно целое, пересчет физики)
  - Если нет - взрыв модуля и станции

```cpp
class Scene {
public:
    explicit Scene(/* config */);
    void update(float dt);  // один физический тик
    void reset();           // перезапуск симуляции
    [[nodiscard]] const PhysicsBody& module_body()  const;
    [[nodiscard]] Autopilot::Phase   phase()        const;
    // ...
private:
    PhysicsBody  module_, station_;
    Gravity      gravity_;
    Atmosphere   atmosphere_;
    Thruster     thruster_;
    Autopilot    autopilot_;
    DockingPort  module_port_, station_port_;
    OrbitConfig  orbit_cfg_;
    float total_time_ = 0.f;
    bool station_docked_ = false;
    void merge_bodies();
};

// Scene.cpp
void Scene::update(float dt) {
	// ── МОДУЛЬ ──────────────────────────────────────────
    // 1. Внешние силы
    module_.apply_force(gravity_.force_on(module_));
    module_.apply_force(atmosphere_.drag_force(module_, totalTime_));

    // 2. Автопилот → управляющие силы
    DockingTarget target = compute_docking_target(
        module_port_, module_, station_port_, station_);
    auto cmd = autopilot_.update(module_, target, dt);
	thruster_.apply_commands(module_, cmd.throttles);

    // 3. Интегрирование + сброс
    module_.integrate(dt);
    module_.reset_forces();
    
    // ── СТАНЦИЯ ─────────────────────────────────────────────────────
    // Станция — полноценное PhysicsBody: движется под действием
    // гравитации и атмосферного сопротивления (ветра).
    // Стыковка не является мгновенной «заморозкой» — станция вместе с модулем продолжают
    // двигаться до фактического объединения тел.
    // DockingTarget пересчитывается КАЖДЫЙ тик (см. выше),
    // поэтому автопилот всегда нацелен на актуальную позицию порта
    if (!station_docked_) {        
	    station_.apply_force(gravity_.force_on(station_));        
	    station_.apply_force(atmosphere_.drag_force(station_, total_time_));     
	    
	    station_.integrate(dt);
	    station_.reset_forces();   
	}

    // ── Стыковка ───────────────────────────────────────── 
    auto result = check_docking(module_port_, module_, station_port_, station_);
    if (result.success) {
	    autopilot_.notify_docked();
	    merge_bodies(); // // ← пересчёт CM, массы, инерции, скоростей
	} else {
	    const bool contact = /* dist(portA, portB) < posThreshold */;
	    if (contact && result.speed > max_docking_speed_)
	        autopilot_.notify_failed();
	}

    total_time_ += dt;
}
```

```cpp
// scene.cpp — Scene::reset() / конструктор
void Scene::reset() {
    const float r = orbit_cfg_.orbit_radius;
    const float v = orbit_cfg_.orbital_speed;

    // Станция — прямо «над» Землёй (в SFML Y↓, «над» = отрицательный Y)
    station_.position = { 0.f, -r };
    station_.velocity = { v,  0.f };   // летит вправо → орбита по часовой стрелке

    // Модуль — рядом, то же состояние орбиты + небольшое смещение
    // Автопилот видит относительную ошибку ~{separation, 0} и начинает сближение
    constexpr float separation = 150.f;  // м, начальное расстояние между портами
    module_.position = { separation, -r };
    module_.velocity = { v, 0.f };       // та же орбитальная скорость

    gravity_ = Gravity{ {0.f, 0.f}, orbit_cfg_.mu() };
}

```

---

## Слой 7 — Рендеринг (`Renderer`)

### Масштаб

```cpp
class Renderer {
public:
    explicit Renderer(float pixels_per_meter = 10.f) 
        : pixels_per_meter_{pixels_per_meter} {}

    void set_scale(float ppm) { pixels_per_meter_ = ppm; }

private:
    float        pixels_per_meter_;
    sf::Vector2f camera_offset_ = {};  // для будущей прокрутки/следования
};

// Физика — в метрах. Renderer — единственное место, где применяется масштаб.
sf::Vector2f to_screen(sf::Vector2f world_pos) {
    return world_pos * pixels_per_meter_ + camera_offset_;
    // + при необходимости смещение камеры
}

// ДОБАВИТЬ в Renderer::draw():
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
[[nodiscard]] constexpr float normalize_angle(float a) noexcept {
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
4. reset_forces()          → сбросить net_force/net_torque
5. Проверить столкновение
6. Проверить докинг
7. Рендерить
```

### 6. `[[nodiscard]]` везде, где важен результат

`check_docking`, `wind_at`, `world_position` — все чистые функции должны быть `[[nodiscard]]`.  
Компилятор предупредит, если результат случайно проигнорирован.

### 7. Residual velocity при откате фазы.
При переходе на более низкую фазу (FINAL → APPROACH → ALIGN) скорость по осям, которые выходят из управления позиционного PID, не обнуляется физически. Для каждой такой оси активировать velocity damper (PID с setpoint = 0) с немедленным включением в момент перехода и ki > 0 для компенсации гравитационного дрейфа.

### 8. Объединение тел после стыковки (Compound Body)

После `check_docking` → `success`: вызвать `Scene::merge_bodies()`.
Пересчитать: массу (сумма), центр масс (взвешенное среднее),
скорость (закон импульса), момент инерции (теорема Штейнера),
угловую скорость (закон момента импульса).
Установить `station_docked_ = true` — модуль исключается из
физического цикла. Compound-тело продолжает движение под
действием гравитации и ветра как единый объект.