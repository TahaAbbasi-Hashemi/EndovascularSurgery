cmake_minimum_required(VERSION 3.2)

project(DVS)

#Important CMAKE stuff
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/lib/cmake)

#VTK
set(VTK_DIR "usr/lib/cmake")
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

#Eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})

#include source code
file(GLOB H_FILES ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB CPP_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(DVS main.cpp ${H_FILES} ${CPP_FILES})
target_link_libraries(DVS ${VTK_LIBRARIES} Eigen3::Eigen)
