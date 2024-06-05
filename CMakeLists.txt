cmake_minimum_required(VERSION 2.8)
project(calicam LANGUAGES C CXX)

find_package(OpenCV REQUIRED)

add_executable(calicam calicam.cpp)
target_link_libraries(calicam ${OpenCV_LIBRARIES})
