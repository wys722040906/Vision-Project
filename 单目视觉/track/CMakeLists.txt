project(track)
cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Debug)

set(OpenCV_DIR "/opt/opencv-4.10/lib/cmake/opencv4")
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

if(OpenCV_FOUND)
    message(STATUS "OpenCV found!")
    message(STATUS "OpenCV version: ${OpenCV_VERSION}")
    message(STATUS "OpenCV include path: ${OpenCV_INCLUDE_DIRS}")
    message(STATUS "OpenCV libraries: ${OpenCV_LIBS}")
else()
    message(FATAL_ERROR "OpenCV not found!")
endif()

#创建
add_library(${PROJECT_NAME} SHARED 
        src/kalmanfilter.cpp
        src/featureMatch.cpp
)


target_include_directories(${PROJECT_NAME} PUBLIC 
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  #构建时指定头文件路径。
        $<INSTALL_INTERFACE:include>                            #安装时指定头文件路径。        
        ${OpenCV_INCLUDE_DIRS}
)    

# 链接库
target_link_libraries(${PROJECT_NAME} PUBLIC 
        ${OpenCV_LIBS}
        Eigen3::Eigen    #Eigen库的头文件路径通过Eigen3::Eigen接口自动设置
        detect
)


#-------cmake package install------------
# 设置安装路径为工作空间的 install 目录
# set(CMAKE_INSTALL_PREFIX "${CMAKE_SOURCE_DIR}/install") -- 单独构建时使用
# 安装目标
install(TARGETS ${PROJECT_NAME}
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include) # 安装头文件

# 安装 CMake 配置文件，供 find_package() 查找
install(EXPORT ${PROJECT_NAME}Targets
        FILE ${PROJECT_NAME}Config.cmake
        NAMESPACE ${PROJECT_NAME}::
        DESTINATION lib/cmake/${PROJECT_NAME})


