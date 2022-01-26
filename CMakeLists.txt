cmake_minimum_required(VERSION 3.2)

project(EndoSurg)

#Important CMAKE stuff
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} /usr/lib/cmake)

#VTK
#set(VTK_DIR "usr/lib/cmake")
find_package(VTK NO_MODULE)
include(${VTK_USE_FILE})

#Eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR})
include_directories(usr/local/include/vtk-8.2)

#include source code
file(GLOB H_FILES ${PROJECT_SOURCE_DIR}/include/*.h)
file(GLOB CPP_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)

add_executable(EndoSurg main.cpp ${H_FILES} ${CPP_FILES})
target_link_libraries(EndoSurg ${VTK_LIBRARIES} Eigen3::Eigen)
