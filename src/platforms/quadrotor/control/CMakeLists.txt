# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add libraries
set(QUADCON_LIB_SRC
    # state estimator
    src/att_estimator.cpp
    # data types
    src/quad_state.cpp
    # controller
    src/att_quat_con.cpp
    src/pos_quat_con.cpp
    # actuator
    src/quad_mixer.cpp    
)
add_library(quad_ctrl STATIC ${QUADCON_LIB_SRC})
target_link_libraries(quad_ctrl librav_if common utilities ${LCM_LIBRARIES})
target_include_directories(quad_ctrl PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  PRIVATE src)

# Add executables
#add_executable(quad_motion_client motion_server/motion_client.cpp)
#target_link_libraries(quad_motion_client ${LCM_LIBRARIES})
