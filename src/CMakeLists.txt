cmake_minimum_required(VERSION 3.10)

# set(CMAKE_FIND_DEBUG_MODE 1)
project(assistedmanipulation)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

# Display search paths for libraries.

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
# find_package(osqp REQUIRED)
find_package(raisim CONFIG REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS} ${pinnocchio_INCLUDE_DIRS})

add_executable(
    test
    test/main.cpp

    controller/filter.cpp
    controller/forecast.cpp
    controller/gram_savitzky_golay/gram_savitzky_golay.cpp
    controller/kalman.cpp
    controller/mppi.cpp
    controller/pid.cpp
    controller/trajectory.cpp
    # controller/qp.cpp

    # test/case/base/reach.cpp
    test/case/base.cpp
    test/case/external_wrench.cpp
    test/case/forecast.cpp
    test/case/trajectory.cpp
    # test/case/pinocchio.cpp

    simulation/frankaridgeback/raisim_dynamics.cpp
    simulation/frankaridgeback/dynamics.cpp
    simulation/frankaridgeback/actor.cpp
    simulation/frankaridgeback/actor_dynamics.cpp
    simulation/simulator.cpp

    frankaridgeback/state.cpp
    frankaridgeback/objective/track_point.cpp
    frankaridgeback/objective/assisted_manipulation.cpp
    frankaridgeback/pinocchio_dynamics.cpp
    frankaridgeback/dynamics.cpp

    logging/assisted_manipulation.cpp
    logging/frankaridgeback.cpp
    logging/mppi.cpp
    logging/pid.cpp
)

target_include_directories(test PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

if (UNIX)
    if(CMAKE_CXX_COMPILER_VERSION VERSION_LESS 10)
        message("gcc version <= 10, currently using version ${CMAKE_CXX_COMPILER_VERSION}")
    endif()

    target_link_libraries(
        test PUBLIC 
        pinocchio::pinocchio
        raisim::raisim
        # osqp::osqp
        nlohmann_json::nlohmann_json
        pthread
    )
elseif (WIN32)
    target_compile_options(test PUBLIC /W3) # /Wall WX

    target_link_libraries(
        test PUBLIC 
        raisim::raisim
        pinocchio::pinocchio
        # osqp::osqp
        nlohmann_json::nlohmann_json
        Ws2_32
    )
    install(FILES $<TARGET_RUNTIME_DLLS:test> DESTINATION bin)
endif()

# Install instructions
install(TARGETS test DESTINATION bin)
install(DIRECTORY frankaridgeback/model DESTINATION bin)
