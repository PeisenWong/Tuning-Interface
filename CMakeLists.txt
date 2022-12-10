cmake_minimum_required(VERSION 3.10)

project(Tune VERSION 1.0)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED True)
set(INC ./inc/CvPlot/cvplot.h)

find_package(OpenCV REQUIRED)
add_subdirectory(serialLib)
# INCLUDE_DIRECTORIES(./inc)

target_include_directories(tune PRIVATE ./inc)
add_executable(tune main.cpp ${INC})
target_link_libraries(tune PUBLIC bluetooth ${OpenCV_LIBS} serialLib)


