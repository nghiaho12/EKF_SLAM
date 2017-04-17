cmake_minimum_required(VERSION 2.8.3)
project(ekf_localization_known_correspondences)

set(CMAKE_CXX_FLAGS "-std=c++11 -O2 ${CMAKE_CXX_FLAGS}")

find_package(OpenGL REQUIRED)

find_package(SDL2 REQUIRED)
# Strip 'SDL2' from the end of the directory, to allow proper namespacing of
# SDL2 include paths.
string(REGEX REPLACE "/SDL2$"
       "" SDL2_INCLUDE_DIRS
       "${SDL2_INCLUDE_DIRS}")


find_package(Eigen3 REQUIRED)

add_executable(EKF_localization_known_correspondences
    src/main.cpp
    src/EKF_SLAM.cpp
    src/robot.cpp
    src/opengl_display.cpp    
)

target_include_directories(EKF_localization_known_correspondences
  PRIVATE
  ${EIGEN3_INCLUDE_DIRS}
  ${SDL2_INCLUDE_DIRS})

target_link_libraries(EKF_localization_known_correspondences
  PRIVATE
  ${SDL2_LIBRARIES}
  ${OPENGL_LIBRARIES})
