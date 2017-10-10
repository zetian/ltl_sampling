cmake_minimum_required(VERSION 2.8.11)
project(acel-examples)

## Include directories
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

## Find LCM
find_package(LCM REQUIRED)

if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)

# Include files for LCM
set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
include_directories(${LCM_GENERATED_HEADER_PATH})

# Add executables
add_executable(lcm_subscriber_cpp lcm_subscriber.cpp)
target_link_libraries(lcm_subscriber_cpp ${LCM_LIBRARIES})

add_executable(lcm_publisher_cpp lcm_publisher.cpp)
target_link_libraries(lcm_publisher_cpp ${LCM_LIBRARIES})

add_executable(logger_example logger_example.cpp)
target_link_libraries(logger_example utility)

add_executable(stopwatch_example stopwatch_example.cpp)
