cmake_minimum_required(VERSION 3.0)
project(ppm)

set(CMAKE_CXX_FLAGS "-Wall -stdlib=libc++ -std=c++1z -O3 ${CMAKE_CXX_FLAGS}")

set(SRC_DIR ${PROJECT_SOURCE_DIR})

set(SRCS 
	${SRC_DIR}/ppm.cpp 
	${SRC_DIR}/bitmap.cpp)

add_executable(ppm ${SRCS})
	

