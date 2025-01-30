cmake_minimum_required(VERSION 3.10)
project(LQR_PID_Controller)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

include_directories(include)


find_package(PythonLibs REQUIRED)
include_directories(${PYTHON_INCLUDE_DIRS})

add_executable(LQR_PID_Controller main_pid.cpp KinematicModel.cpp LQR.cpp Reference_path.cpp pid_control.cpp)
target_link_libraries(LQR_PID_Controller ${PYTHON_LIBRARIES})
