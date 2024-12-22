project(KalmanFilter)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")

find_package(Eigen3 REQUIRED)

# include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# add_library(KalmanFilter ${CMAKE_CURRENT_SOURCE_DIR}/src/KalmanFilter.cpp)

add_executable(KalmanFilter ${CMAKE_CURRENT_SOURCE_DIR}/test5.cpp)
target_include_directories(KalmanFilter PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(KalmanFilter Eigen3::Eigen)