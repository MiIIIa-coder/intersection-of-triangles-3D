cmake_minimum_required(VERSION 3.16.3)
project(triangle_intersec)

set(CMAKE_CXX_STANDARD 17)

set(SOURCE_EXE triangle.cpp)

set(SOURCE_LIB geom_obj.cpp octree.cpp)

add_library(lib STATIC ${SOURCE_LIB})

add_executable(${PROJECT_NAME} ${SOURCE_EXE})

target_link_libraries(${PROJECT_NAME} lib)