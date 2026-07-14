cmake_minimum_required(VERSION 3.17)
project(teb)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Boost REQUIRED COMPONENTS system thread graph)
find_package(SUITESPARSE REQUIRED)
find_package(G2O REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Concurrent)

include_directories(
        ${PROJECT_SOURCE_DIR}/inc
        /usr/include/eigen3
        /usr/include/nlohmann
        ${SUITESPARSE_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIR}
        ${Qt5Widgets_INCLUDE_DIRS}
)

add_library(lib_teb
        src/teb_config.cpp
        src/obstacles.cpp
        src/visualization.cpp
        src/optimal_planner.cpp
        src/timed_elastic_band.cpp
        src/occupancy_grid.cpp
        src/teb_scene.cpp
)

add_executable(teb main.cpp)

target_link_libraries(
        teb
        lib_teb
        ${SUITESPARSE_LIBRARIES}
        ${G2O_LIBRARIES}
        -lboost_system -lboost_thread -lboost_graph
        Qt5::Widgets
        Qt5::Concurrent
)

add_executable(test_teb tests/test_teb.cpp)

target_link_libraries(
        test_teb
        lib_teb
        ${SUITESPARSE_LIBRARIES}
        ${G2O_LIBRARIES}
        -lboost_system -lboost_thread -lboost_graph
        Qt5::Widgets
        Qt5::Concurrent
)

add_executable(test_teb_scene tests/test_teb_scene.cpp)

target_link_libraries(
        test_teb_scene
        lib_teb
        ${SUITESPARSE_LIBRARIES}
        ${G2O_LIBRARIES}
        -lboost_system -lboost_thread -lboost_graph
        Qt5::Widgets
)

enable_testing()
add_test(NAME teb_scene_round_trip COMMAND test_teb_scene)
