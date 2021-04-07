cmake_minimum_required (VERSION 2.8)
project (MiniRenderer)

if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "Debug")
endif()

set (INCLUDE_DIR "${PROJECT_SOURCE_DIR}/include")
set (SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")
file(GLOB_RECURSE SOURCE_FILES "${SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE INCLUDE_FILES "${INCLUDE_DIR}/*.h")

if (MSVC)

add_compile_options("/std:c++17")

else()

set (CC_FLAGS_DEBUG "-g -std=c++17")
set (CC_FLAGS_RELEASE "-O3 -std=c++17")

endif(MSVC)

set (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} ${CC_FLAGS_DEBUG}")

set (CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} ${CC_FLAGS_RELEASE}")

message (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")

source_group(TREE ${SOURCE_DIR} FILES ${SOURCE_FILES})
source_group(TREE ${INCLUDE_DIR} FILES ${INCLUDE_FILES})

add_executable(MiniRenderer ${SOURCE_FILES})

target_include_directories(MiniRenderer PUBLIC ${INCLUDE_DIR})