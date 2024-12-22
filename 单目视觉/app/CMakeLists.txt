project(app)
cmake_minimum_required(VERSION 3.10)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)  

#实用之前先把staticLib和sharedLib编译好，并安装到指定目录
# set(CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/install/lib/cmake")  # 设置搜索路径
# find_package(staticLib REQUIRED)
# find_package(sharedLib REQUIRED)
set(OpenCV_DIR "/opt/opencv-4.10/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)



add_executable(${PROJECT_NAME} 
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
)

target_link_libraries(${PROJECT_NAME} PUBLIC 
    detect
    ${OpenCV_LIBS}
    track
    predict
    Eigen3::Eigen
)

target_include_directories(${PROJECT_NAME} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${detect_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${track_INCLUDE_DIRS}
)

