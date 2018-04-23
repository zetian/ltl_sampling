cmake_minimum_required(VERSION 2.8.11)
project(librav_quadrotor)

## Set compiler to use c++ 11 features
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX14)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX14)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++14 or C++11 support. Please use a different C++ compiler.")
endif()

## Put all binary files into /bin and libraries into /lib
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

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

## Find Gurobi
find_package(GUROBI QUIET)
if (NOT GUROBI_FOUND)
  message("-- Gurobi not found. Trajectory generation will not be built. Make sure you have set the GUROBI_HOME variable")
else()
  message("-- Gurobi found")
  include_directories(${GUROBI_INCLUDE_DIRS})
endif()

# set(QUAD_BASE_SRC
#     driver/quad_data_broadcaster.cpp    
#     motion_server/motion_server.cpp
#     motion_server/quad_flattraj_handler.cpp
#     mission/mission_tracker.cpp
# )
# set(TRAJ_SRC     
#     mission/path_manager.cpp
#     motion_server/quad_polyopt.cpp
# )

# if (NOT GUROBI_FOUND)
#   set(QUAD_MOD_SRC ${QUAD_BASE_SRC})
# else()
#   set(QUAD_MOD_SRC ${QUAD_BASE_SRC} ${TRAJ_SRC})
# endif()

# add_library(quad_base STATIC ${QUAD_MOD_SRC})
# if (NOT GUROBI_FOUND)
#   target_link_libraries(quad_base common geometry ${LCM_LIBRARIES})
# else()
#   target_link_libraries(quad_base polyopt common geometry ${LCM_LIBRARIES})
# endif()

## Add other sub source directories
#add_subdirectory(apps)
#add_subdirectory(control)
#add_subdirectory(system)

# Tests
# add_subdirectory(tests)
