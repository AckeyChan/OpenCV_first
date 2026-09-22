cmake_minimum_required(VERSION 3.16)
project(OpenCV_first LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Ceres REQUIRED)
add_executable(OpenCV_first src/main.cpp)

target_include_directories(OpenCV_first PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(OpenCV_first PRIVATE ${OpenCV_LIBS} Eigen3::Eigen Ceres::ceres)