# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

## Find Gurobi
find_package(GUROBI QUIET)
if (NOT GUROBI_FOUND)
  message("-- Gurobi not found. Trajectory generation will not be built. Make sure you have set the GUROBI_HOME variable")
else()
  message("-- Gurobi found")
  include_directories(${GUROBI_INCLUDE_DIRS})
endif()

# Add libraries
set(QUAD_BASE_SRC
    src/quad_data_broadcaster.cpp    
    src/motion_server.cpp
    src/quad_flattraj_handler.cpp
    src/mission_tracker.cpp
)
set(TRAJ_SRC     
    src/path_manager.cpp
    src/quad_polyopt.cpp
)
if (NOT GUROBI_FOUND)
  set(QUAD_SYS_SRC ${QUAD_BASE_SRC})
else()
  set(QUAD_SYS_SRC ${QUAD_BASE_SRC} ${TRAJ_SRC})
endif()
add_library(quad_sys STATIC ${QUAD_SYS_SRC})
if (NOT GUROBI_FOUND)
  target_link_libraries(quad_sys quad_ctrl common ${LCM_LIBRARIES})
else()
  target_link_libraries(quad_sys quad_ctrl polyopt common ${LCM_LIBRARIES})
endif()
target_include_directories(quad_sys PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
#add_executable(quad_motion_client motion_server/motion_client.cpp)
#target_link_libraries(quad_motion_client ${LCM_LIBRARIES})
