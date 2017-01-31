cmake_minimum_required(VERSION 3.0)
project(arSLAM)

#set(CMAKE_CXX_STANDARD 11)
#
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3   -lopencv_aruco")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3   -lopencv_aruco")

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

# g2o
 add_subdirectory(${PROJECT_SOURCE_DIR}/thirdparty/g2o)
#include_directories()
#link_directories(${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib)
# G2O
SET(G2O_ROOT "/usr/local/include/g2o")
FIND_PACKAGE(G2O REQUIRED)
FIND_PACKAGE( CSparse REQUIRED)
#IF(G2O_FOUND)
#
#    MESSAGE("FIND g2o")
#ELSE(G2O_FOUND)
#    MESSAGE(FATAL_ERROR "g2o not FOUND."
#ENDIF(G2O_FOUND)
SET(G2O_LIB_TYPE SHARE)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ./include
        /usr/local/cuda/include
        /usr/include
        /usr/local/include
                ${CSPARSE_INCLUDE_DIR}
#     ${PROJECT_SOURCE_DIR}/thirdparty/g2o
 ${PROJECT_SOURCE_DIR}/thirdparty/g2o
        # /usr/include/suitesparse

)

#aux_source_directory(
#        ./src
#)


link_directories(
        /usr/local/lib
        /usr/lib
        ${PCL_LIBRARY_DIRS}
#        ~/Code/g2o/libCMakeLists.txt:60
${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib
)
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )


add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(ArPoseTest ArPoseTest.cpp)
target_link_libraries(ArPoseTest ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(VisualTest visualTest.cpp)
target_link_libraries(VisualTest ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(gwotest g2oTest.cpp)
target_link_libraries(gwotest  ${OpenCV_LIBS} )
#target_link_libraries(gwotest ${G2O_LIBS} ${OpenCV_LIBS} )

