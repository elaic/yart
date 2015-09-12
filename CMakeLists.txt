cmake_minimum_required(VERSION 3.0)
project(rt)

set(CMAKE_CXX_FLAGS "-Wall -Wpedantic -stdlib=libc++ -std=c++1z -O3 -g ${CMAKE_CXX_FLAGS}")
# set(CMAKE_EXE_LINKER_FLAGS "-lprofiler -Wl,-no_pie ${CMAKE_EXE_LINKER_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(INCL ${PROJECT_SOURCE_DIR}/include)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

set(SRCS
	${SRC_DIR}/rt.cpp
	${SRC_DIR}/bitmap.cpp
	${SRC_DIR}/scheduler.cpp)

include_directories(${INCL})
add_executable(rt ${SRCS})


