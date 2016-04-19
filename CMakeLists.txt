cmake_minimum_required(VERSION 3.0)
project(rt)

if (UNIX)
	# -fno-rtti - disables rtti
	# -Werror - add when spectrum class is implemented
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-rtti -Werror -Wall -Wpedantic -std=c++1z -O3 -g")
    if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpthread")
    endif()
elseif (MSVC)
	# /GR- - disables rtti
	# /sdl - additional checks, add when spectrum class is implemented
	# /Wx - treat warnings as errors, same as above
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR- /W4 /sdl /WX /arch:AVX")
else()
	message(FATAL_ERROR "Unsupported system")
endif()
# set(CMAKE_EXE_LINKER_FLAGS "-lprofiler -Wl,-no_pie ${CMAKE_EXE_LINKER_FLAGS}")

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

# project dependencies
include_directories(${PROJECT_SOURCE_DIR}/external/tinyobjloader)
set(EXTERNAL_SRCS
	${PROJECT_SOURCE_DIR}/external/tinyobjloader/tiny_obj_loader.h
	${PROJECT_SOURCE_DIR}/external/tinyobjloader/tiny_obj_loader.cc)

set(INCL ${PROJECT_SOURCE_DIR}/include)
set(SRC_DIR ${PROJECT_SOURCE_DIR}/src)

set(INCLUDES
	${INCL}/bitmap.h
	${INCL}/bsdf.h
	${INCL}/camera.h
	${INCL}/constants.h
	${INCL}/frame.h
	${INCL}/light.h
    ${INCL}/platform.h
    ${INCL}/qmc.h
	${INCL}/range.h
	${INCL}/renderer.h
	${INCL}/rng.h
	${INCL}/scene.h
	${INCL}/scheduler.h
	${INCL}/semaphore.h
	${INCL}/shape.h
	${INCL}/spectrum.h
	${INCL}/sphere.h
	${INCL}/timer.h
	${INCL}/triaccel.h
	${INCL}/triangle.h
	${INCL}/utils.h
	${INCL}/vector.h)

set(SRCS
	${SRC_DIR}/rt.cpp
	${SRC_DIR}/bitmap.cpp
	${SRC_DIR}/bsdf.cpp
	${SRC_DIR}/renderer.cpp
	${SRC_DIR}/scene.cpp
	${SRC_DIR}/scheduler.cpp)

include_directories(${INCL})
add_executable(rt ${SRCS} ${INCLUDES} ${EXTERNAL_SRCS})

source_group("include" FILES ${INCLUDES})
source_group("src" FILES ${SRCS})
source_group("external\\tinyobjloader" FILES ${EXTERNAL_SRCS})
