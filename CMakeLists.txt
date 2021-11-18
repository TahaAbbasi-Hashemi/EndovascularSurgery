cmake_minimum_required(VERSION 3.2)

project(DVS)

#Important CMAKE stuff
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

#VTK
set(VTK_DIR "usr/local/lib")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

#Eigen
#find_package (Eigen3 3.3 REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
#set(OpenVDB_DIR "usr/local/lib")
#find_package(OpenVDB REQUIRED)

#Dynamixel
#link_directories(/home/dvs/DynamixelSDK/c++/build/linux64)
#include_directories("/home/dvs/DynamixelSDK/c++/build/linux64")
#include_directories("/home/dvs/DynamixelSDK/c++/include/dynamixel_sdk")
#find_library(DynamixelSDK_LIB DynamixelSDK)

#include directories
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})
#include_directories(usr/local/include/vtk-8.2)
#include_directories(/usr/include/vtk)
#include_directories(/usr/include)

#include source code
file(GLOB H_FILES ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB CPP_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(DVS main.cpp ${H_FILES} ${CPP_FILES})
target_link_libraries(DVS ${VTK_LIBRARIES} Eigen3::Eigen)
