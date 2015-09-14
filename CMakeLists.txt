cmake_minimum_required(VERSION 3.0)
project(rt)

if (UNIX)
	# -fno-rtti - disables rtti
	set(CMAKE_CXX_FLAGS "-Wall -Wpedantic -stdlib=libc++ -std=c++1z -O3 -g ${CMAKE_CXX_FLAGS}")
elseif (MSVC)
	# /GR- - disables rtti
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W4 /sdl")
	message("${CMAKE_CXX_FLAGS}")
else()
	message(FATAL_ERROR "Unsupported system")
endif()
# set(CMAKE_EXE_LINKER_FLAGS "-lprofiler -Wl,-no_pie ${CMAKE_EXE_LINKER_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

set(INCL ${PROJECT_SOURCE_DIR}/include)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

set(INCLUDES
	${INCL}/bitmap.h
	${INCL}/bsdf.h
	${INCL}/camera.h
	${INCL}/constants.h
	${INCL}/frame.h
	${INCL}/light.h
	${INCL}/rng.h
	${INCL}/scheduler.h
	${INCL}/triangle.h
	${INCL}/vector.h)

set(SRCS
	${SRC_DIR}/rt.cpp
	${SRC_DIR}/bitmap.cpp
	${SRC_DIR}/scheduler.cpp)

include_directories(${INCL})
add_executable(rt ${SRCS} ${INCLUDES})

source_group("include" FILES ${INCLUDES})
source_group("src" FILES ${SRCS})
