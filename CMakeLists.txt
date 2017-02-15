cmake_minimum_required(VERSION 3.0)
project(arSLAM)

#set(CMAKE_CXX_STANDARD 11)
#
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3   -lopencv_aruco -lpthread ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -O3   -lopencv_aruco")


find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
add_definitions(${PCL_DEFINITIONS})
FIND_PACKAGE(OpenMP REQUIRED)
if (OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif ()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
# G2O
SET(G2O_ROOT /usr/local/include/g2o)
FIND_PACKAGE(G2O)
FIND_PACKAGE(CSparse)

INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})
#IF(G2O_FOUND)
#
#    MESSAGE("FIND g2o")
#ELSE(G2O_FOUND)
#    MESSAGE(FATAL_ERROR "g2o not FOUND."
#ENDIF(G2O_FOUND)
#SET(G2O_LIB_TYPE SHARE)
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ./include
        /usr/local/cuda/include
        /usr/include
        /usr/local/include
        ${CSPARSE_INCLUDE_DIR}
        ${G2O_INCLUDE_DIR}
#     ${PROJECT_SOURCE_DIR}/thirdparty/g2o
# ${PROJECT_SOURCE_DIR}/thirdparty/g2o
        # /usr/include/suitesparse
        ./FilterLib
        ~/Cpp_Extent/Cpp_Extent

)

#aux_source_directory(
#        ./src
#)


link_directories(
        /usr/local/lib
        /usr/lib
        ${PCL_LIBRARY_DIRS}
        #        ~/Code/g2o/libCMakeLists.txt:60
        #${PROJECT_SOURCE_DIR}/thirdparty/g2o/lib
)
SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator
        g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
        g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only
        g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper
        g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg
        g2o_types_data g2o_types_sim3 cxsparse
        g2o_types_slam3d_addons)


add_executable(main main.cpp)
target_link_libraries(main ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(ArPoseTest ArPoseTest.cpp)
target_link_libraries(ArPoseTest ${G2O_LIBS} ${CSPARSE_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(VisualTest visualTest.cpp)
target_link_libraries(VisualTest ${OpenCV_LIBS} ${PCL_LIBRARIES})


add_executable(gwotest g2oTest.cpp)
#target_link_libraries(gwotest  ${OpenCV_LIBS} )
target_link_libraries(gwotest ${G2O_LIBS} ${CSPARSE_LIBRARY} ${OpenCV_LIBS})

add_executable(G2O_Solve G2O_Solve.cpp)
target_link_libraries(G2O_Solve ${G2O_LIBS} ${CSPARSE_LIBRARY} ${OpenCV_LIBS} ${PCL_LIBRARIES})

