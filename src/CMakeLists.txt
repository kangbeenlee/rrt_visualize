cmake_minimum_required(VERSION 3.8)
project(src)

set(CMAKE_CXX_STANDARD 11)

add_executable(src RRT.cpp RRT.h Grid_map.cpp Grid_map.h)

link_directories(/usr/local/include)
