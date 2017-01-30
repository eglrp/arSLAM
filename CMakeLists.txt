cmake_minimum_required(VERSION 3.0)
project(arSLAM)

set(CMAKE_CXX_STANDARD 11)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3   -lopencv_aruco")

LIST(APPEND CMAKE_MODULE_PATH ./cmake_modules/  /home/steve/Code/arSLAM/cmake_modules/)

#
#list(APPEND CMAKE_MODULE_PATH usr/share/OpenCV/
#        )
#set(OPENCV_DIR /opt/ros/jada/share/OpenCV-3.1.0-dev/)
#find_package(VTK REQUIRED) #Just for vtk5 ... I don't know why...
find_package(Eigen3 REQUIRED)
#find_package(OPENCV REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL  REQUIRED)
add_definitions(${PCL_DEFINITIONS})

# G2O
SET(G2O_ROOT "/usr/local/include/g2o")

FIND_PACKAGE(G2O)
IF(G2O_FOUND)

ELSE(G2O_FOUND)
    MESSAGE(FATAL_ERROR "g2o not FOUND.")
ENDIF(G2O_FOUND)

include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ./include
        /usr/local/cuda/include
        /usr/include
#        /usr/local/include
#        ~/Code/g2o/g2o

)

#aux_source_directory(
#        ./src
#)


link_directories(
        /usr/local/lib
        /usr/lib
        ${PCL_LIBRARY_DIRS}
#        ~/Code/g2o/lib
)

add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(ArPoseTest ArPoseTest.cpp)
target_link_libraries(ArPoseTest ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(VisualTest visualTest.cpp)
target_link_libraries(VisualTest ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(gwotest g2oTest.cpp)
target_link_libraries(gwotest ${OpenCV_LIBS})

