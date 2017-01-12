cmake_minimum_required(VERSION 3.6)
project(arSLAM)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3  -pthread -lopencv_aruco")
#
#list(APPEND CMAKE_MODULE_PATH usr/share/OpenCV/
#        )
#set(OPENCV_DIR /opt/ros/jada/share/OpenCV-3.1.0-dev/)

find_package(Eigen3 REQUIRED)
#find_package(OPENCV REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ./include
        /usr/local/cuda/include
        /usr/include
        /usr/local/include
)


link_directories(
        /usr/local/lib
        /usr/lib
)

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS})

