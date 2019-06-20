cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

rosbuild_init()

# IF(NOT ROS_BUILD_TYPE)
  SET(ROS_BUILD_TYPE Release)
# ENDIF()

MESSAGE("Build type: " ${ROS_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

# Check C++11 or C++0x support
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
   add_definitions(-DCOMPILEDWITHC11)
   message(STATUS "Using flag -std=c++11.")
elseif(COMPILER_SUPPORTS_CXX0X)
   set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
   add_definitions(-DCOMPILEDWITHC0X)
   message(STATUS "Using flag -std=c++0x.")
else()
   message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../../cmake_modules)

# customize opencv dir
 if ($ENV{ROS_DISTRO} STREQUAL "indigo")
     set( OpenCV_DIR /opt/opencv2/share/OpenCV )
     find_package(OpenCV 2.4.13 REQUIRED core ocl)
 elseif ($ENV{ROS_DISTRO} STREQUAL "kinetic")
     set( OpenCV_DIR /opt/opencv-3.4.1/share/OpenCV )
     set( OpenCV_DIR /opt/opencv3/share/OpenCV )
     find_package(OpenCV 3.4.0 REQUIRED)
 endif()

# find_package(Eigen3 3.1.0 REQUIRED)
set( EIGEN3_INCLUDE_DIR /usr/include/eigen3 )
MESSAGE("EIGEN include dir: " ${EIGEN3_INCLUDE_DIR})

# set( Pangolin_DIR /opt/Pangolin/lib/cmake/Pangolin/ )
# find_package(Pangolin REQUIRED)
set(Pangolin_INCLUDE_DIRS /opt/Pangolin/include/)
set(Pangolin_LIBRARIES /opt/Pangolin/lib/libpangolin.so)
MESSAGE("Pangolin include dir: " ${Pangolin_INCLUDE_DIRS})

find_package(Boost COMPONENTS system)

include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../../
${PROJECT_SOURCE_DIR}/../../../include
${Pangolin_INCLUDE_DIRS}
${EIGEN3_INCLUDE_DIR}
${Boost_INCLUDE_DIRS}
)

set(LIBS 
	GL  
    GLU
    GLEW
    glut
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${PROJECT_SOURCE_DIR}/../../../Thirdparty/DBoW2/lib/libDBoW2.so
${PROJECT_SOURCE_DIR}/../../../Thirdparty/g2o/lib/libg2o.so
${PROJECT_SOURCE_DIR}/../../../lib/libORB_SLAM2.so
${Boost_LIBRARIES}
)

# Node for monocular camera
rosbuild_add_executable(Mono
src/ros_mono.cc
)

target_link_libraries(Mono
${LIBS}
)

# Node for monocular camera (Augmented Reality Demo)
rosbuild_add_executable(MonoAR
src/AR/ros_mono_ar.cc
src/AR/ViewerAR.h
src/AR/ViewerAR.cc
)

target_link_libraries(MonoAR
${LIBS}
)

# Node for stereo camera
rosbuild_add_executable(Stereo
src/ros_stereo.cc
)

target_link_libraries(Stereo
${LIBS}
)

# Node for RGB-D camera
rosbuild_add_executable(RGBD
src/ros_rgbd.cc
)

target_link_libraries(RGBD
${LIBS}
)

