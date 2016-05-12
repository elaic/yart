cmake_minimum_required(VERSION 3.0)
project(rt)

# This is ugly...
# Determine AVX support. If no AVX support found, fail with fatal error
try_run(
    AVX_SUPPORT CPUID_COMPILE_RESULT
    ${CMAKE_BINARY_DIR}
    ${PROJECT_SOURCE_DIR}/cmake/cpuid.c
    CMAKE_FLAGS "-DCMAKE_C_FLAGS:STRING=-std=c11")

set(CXX_AVX_DEFINE_FLAGS "")
set(CXX_AVX_COMPILER_FLAGS "")

set(AVX 1)
set(AVX2 2)
set(AVX_FMA 3)
set(AVX2_FMA 4)

if (${AVX_SUPPORT} EQUAL ${AVX})
    message(STATUS "AVX support detected")
    if (UNIX)
        set(CXX_AVX_COMPILER_FLAGS " -mavx")
        set(CXX_AVX_DEFINE_FLAGS " -DYART_AVX")
    elseif (MSVC)
        set(CXX_AVX_COMPILER_FLAGS " /arch:AVX")
        set(CXX_AVX_DEFINE_FLAGS " /DYART_AVX")
    endif()
elseif(${AVX_SUPPORT} EQUAL ${AVX2})
    message(STATUS "AVX2 support detected")
    if (UNIX)
        set(CXX_AVX_COMPILER_FLAGS " -mavx -mavx2")
        set(CXX_AVX_DEFINE_FLAGS " -DYART_AVX -DYART_AVX2")
    elseif (MSVC)
        set(CXX_AVX_COMPILER_FLAGS " /arch:AVX2")
        set(CXX_AVX_DEFINE_FLAGS " /DYART_AVX /DYART_AVX2")
    endif()
elseif (${AVX_SUPPORT} EQUAL ${AVX_FMA})
    message(STATUS "AVX support with FMA detected")
    if (UNIX)
        set(CXX_AVX_COMPILER_FLAGS " -mavx -mfma")
        set(CXX_AVX_DEFINE_FLAGS " -DYART_AVX -DYART_FMA")
    elseif (MSVC)
        set(CXX_AVX_COMPILER_FLAGS " /arch:AVX")
        set(CXX_AVX_DEFINE_FLAGS " /DYART_AVX /DYART_FMA")
    endif()
elseif (${AVX_SUPPORT} EQUAL ${AVX2_FMA})
    message(STATUS "AVX2 support with FMA detected")
    if (UNIX)
        set(CXX_AVX_COMPILER_FLAGS " -mavx -mavx2 -fma")
        set(CXX_AVX_DEFINE_FLAGS " -DYART_AVX -DYART_AVX2 -DYART_FMA")
    elseif (MSVC)
        set(CXX_AVX_COMPILER_FLAGS " /arch:AVX2")
        set(CXX_AVX_DEFINE_FLAGS " /DYART_AVX /DYART_AVX2 /DYART_FMA")
    endif()
else()
    message(FATAL_ERROR "No AVX support detected, fatal error.")
endif()

if (UNIX)
	# -fno-rtti - disables rtti
	# -Werror - add when spectrum class is implemented
    set(CMAKE_CXX_FLAGS
        "${CMAKE_CXX_FLAGS} -fno-rtti -Werror -Wall -Wpedantic \
        -std=c++1z -O3 -DNDEBUG -g \
        ${CXX_AVX_DEFINE_FLAGS} \
        ${CXX_AVX_COMPILER_FLAGS}")
    if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
        set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -lpthread")
    endif()
elseif (MSVC)
	# /GR- - disables rtti
	# /sdl - additional checks, add when spectrum class is implemented
	# /Wx - treat warnings as errors, same as above
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /GR- /W4 /sdl /WX \
    ${CXX_AVX_DEFINE_FLAGS} \
    ${CXX_AVX_COMPILER_FLAGS}")
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
	${INCL}/vector.h
	${INCL}/vector8.h)

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
