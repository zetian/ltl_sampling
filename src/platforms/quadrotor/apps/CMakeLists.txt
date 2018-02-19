# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_SOURCE_DIR}/src)

# Dependency libraries
#find_package(LIBRARY_NAME REQUIRED)

# Add executables
# add_executable(path_repair_pysim path_repair_pysim.cpp)
# target_link_libraries(path_repair_pysim map pr_sim quad_base common ${LCM_LIBRARIES})

# add_executable(quadsim_pr_planner quadsim_planner.cpp)
# target_link_libraries(quadsim_pr_planner map path_repair quad_base common ${LCM_LIBRARIES})

# add_executable(planar_search planar_search.cpp)
# target_link_libraries(planar_search geometry map vis)

# add_executable(map_gen map_gen.cpp)
# target_link_libraries(map_gen map vis)

# add_executable(map_param map_param.cpp)
# target_link_libraries(map_param geometry vis ${LCM_LIBRARIES})

# add_executable(local_analysis local_analysis.cpp)
# target_link_libraries(local_analysis map vis pr_sim)

# add_executable(octomap_server_node octomap_server_node.cpp)
# target_link_libraries(octomap_server_node pr_sim map geometry common ${Octomap_LIBS} ${LCM_LIBRARIES})

# add_executable(shortcut_analysis shortcut_analysis.cpp)
# target_link_libraries(shortcut_analysis map vis pr_sim)
