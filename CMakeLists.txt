cmake_minimum_required(VERSION 2.8)
project(evaluate_pose)

#OpenCV 3.0
find_package(OpenCV 3.0 REQUIRED)

#Eigen版本我使用的为3.3.5（使用绝对路径）
include_directories(
        "/usr/local/include/eigen3/"
)
#Sophus（使用绝对路径）
#https://github.com/strasdat/Sophus/tree/13fb3288311485dc94e3226b69c9b59cd06ff94e
find_package(Sophus REQUIRED)
include_directories(
        "/usr/local/include/sophus/"
)

#Pangolin
find_package(Pangolin REQUIRED)
include_directories(${Pangolin_INCLUDE_DIRS})

#CMAKE
set(CMAKE_CXX_STANDARD 11)

add_executable(evaluate_pose main.cpp)
target_link_libraries(evaluate_pose ${Pangolin_LIBRARIES})