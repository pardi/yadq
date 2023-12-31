cmake_minimum_required(VERSION 3.10)
project(yadq)

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel.")


if(CMAKE_BUILD_TYPE STREQUAL "Release")
    # Set compiler flags for optimization
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++17")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
endif()

# Options
option(BUILD_TESTS "Build the project tests" OFF)

# Collect files
file(GLOB HEADER_FILES include/*.hpp)
list(REMOVE_ITEM HEADER_FILES "include/yadq/yadq_type_traits.hpp")
file(GLOB TESTS_FILES tests/*.cpp)

# Create library as interface
set(LIB_NAME "${PROJECT_NAME}")

add_library(${LIB_NAME} INTERFACE)
target_include_directories(${LIB_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

# Simple testing main
add_executable(main 
src/main.cpp)

target_compile_options(main PRIVATE -Wall -Wextra)
target_link_libraries(main PRIVATE ${LIB_NAME})

if(BUILD_TESTS) 
  # Tests
  add_subdirectory(deps/gtest)
  enable_testing()

  add_executable(
    tests
    ${TESTS_FILES}
    )

  target_link_libraries(tests
    GTest::gtest_main
    ${LIB_NAME}
  )

  include(GoogleTest)
  gtest_discover_tests(tests)
endif()

