cmake_minimum_required( VERSION 2.8 )
project(AutonomousVehicle)
find_package(Qt4 REQUIRED)

include_directories(inc)
set(Dynamics_SOURCES_PATH src)

set(Dynamics_SOURCES ${Dynamics_SOURCES_PATH}/main.cpp
${Dynamics_SOURCES_PATH}/gui.cpp
${Dynamics_SOURCES_PATH}/symulacja.cpp
${Dynamics_SOURCES_PATH}/Vehicle.cpp
${Dynamics_SOURCES_PATH}/VehicleVisualization.cpp
${Dynamics_SOURCES_PATH}/wahadlo.cpp
${Dynamics_SOURCES_PATH}/WizualizacjaWahadla.cpp)

set(Dynamics_HEADERS inc/VehicleVisualization.hh
inc/Vehicle.hh
inc/WizualizacjaWahadla.hh
inc/wahadlo.hh
inc/gui.hh)

qt4_wrap_cpp(Dynamics_HEADERS_MOC ${Dynamics_HEADERS})

include(${QT_USE_FILE})
add_definitions(${QT_DEFINITIONS})

add_executable(Dynamics ${Dynamics_SOURCES} ${Dynamics_HEADERS_MOC})
target_link_libraries(Dynamics ${QT_LIBRARIES})

