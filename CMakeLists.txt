cmake_minimum_required(VERSION 3.10)
project(Quaternion)

# if(CMAKE_COMPILER_IS_GNUCXX OR ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang"))
#     set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
# endif()
set(CMAKE_CXX_FLAGS "-std=c++17 ${CMAKE_CXX_FLAGS} -O3")

set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel.")


# if(CMAKE_BUILD_TYPE STREQUAL "Release")
#     # Set compiler flags for optimization
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
#     set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -O3")
# endif()

add_executable(${PROJECT_NAME} 
main.cpp 
quaternion.cpp)

target_include_directories(${PROJECT_NAME}  PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(${PROJECT_NAME}  PRIVATE -Wall -Wextra)

target_link_libraries(${PROJECT_NAME} PRIVATE)