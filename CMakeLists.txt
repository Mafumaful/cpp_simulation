cmake_minimum_required(VERSION 3.0.0)
set(CMAKE_BUILD_TYPE "Debug")
project(simulator)
add_compile_options(-std=c++11)

find_package(Eigen3 REQUIRED)
find_package(PythonLibs 3.6 REQUIRED)

include_directories(
    ${EIGEN3_INCLUDE_DIR} 
    ${PYTHON_INCLUDE_DIRS}
    include/controller_lib 
    include/filter_lib 
    include/plot_lib
    )

add_executable(filter_average src/filter_average.cpp)
target_link_libraries(filter_average ${PYTHON_LIBRARIES})