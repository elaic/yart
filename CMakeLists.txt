cmake_minimum_required(VERSION 3.0)
project(ppm)

# set(CMAKE_CXX_COMPILER "clang-3.7")
set(CMAKE_CXX_FLAGS "-Wall -Wpedantic -stdlib=libc++ -std=c++1z -O3 -g ${CMAKE_CXX_FLAGS}")
# set(CMAKE_EXE_LINKER_FLAGS "-lprofiler -Wl,-no_pie ${CMAKE_EXE_LINKER_FLAGS}")

set(SRC_DIR ${PROJECT_SOURCE_DIR})

set(SRCS
	${SRC_DIR}/ppm.cpp
	${SRC_DIR}/bitmap.cpp
	${SRC_DIR}/scheduler.cpp)

add_executable(ppm ${SRCS})


