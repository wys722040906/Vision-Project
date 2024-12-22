project(MyWorkspace)
cmake_minimum_required(VERSION 3.10)


#顶层 CMakeLists.txt 中使用 add_subdirectory()  + 库 和 app
#不需要 find_package()，但会将 staticLib 作为直接依赖构建
#find_package(staticLib REQUIRED) //需要安装到系统路径

# 设置 OpenCV 路径（根据你 OpenCV 安装路径配置）
set(OpenCV_DIR "/opt/opencv-4.10/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)



add_subdirectory(detect)
add_subdirectory(app)
add_subdirectory(track)
add_subdirectory(predict)




#设置安装路径--所有包--add_subdirectory()--全局设置
set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install")  
#  cmake -B build  |  cmake --build build | camke --install build


# 单独构建库--顶层 CMakeLists.txt 中使用 add_subdirectory()  + 库
# cmake -S staticLib1 -B build/staticLib1 -DCMAKE_INSTALL_PREFIX="${CMAKE_SOURCE_DIR}/app1/install"
# cmake --build build/staticLib1
# cmake --install build/staticLib1
