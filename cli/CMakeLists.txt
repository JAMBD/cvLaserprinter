cmake_minimum_required(VERSION 2.8)
project( cvLaser )
find_package( OpenCV REQUIRED )
list( APPEND CMAKE_CXX_FLAGS "-std=c++0x ${CMAKE_CXX_FLAGS} -O2 -g -ftest-coverage -fprofile-arcs")

add_executable( cvLaser cvLaser.cpp )

target_link_libraries( cvLaser ${OpenCV_LIBS} )
