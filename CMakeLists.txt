cmake_minimum_required(VERSION 2.8.3)
project(ekf_localization_known_correspondences)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")

find_package(OpenGL REQUIRED)
find_package(SDL2 REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(EKF_localization_known_correspondences
    src/main.cpp
    src/EKF_localization.cpp
    src/robot.cpp
    src/opengl_display.cpp    
)

include_directories(EIGEN3_INCLUDE_DIR)
include_directories(/usr/include/eigen3)

target_link_libraries(EKF_localization_known_correspondences ${SDL2_LIBRARIES} ${OPENGL_LIBRARIES})
