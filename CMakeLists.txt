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
    src/control/Autopilot.cpp
    src/rendering/Renderer.cpp
)

target_link_libraries(docking_sim PRIVATE sfml-graphics sfml-window sfml-system)
target_compile_options(docking_sim PRIVATE -Wall -Wextra -Wpedantic)