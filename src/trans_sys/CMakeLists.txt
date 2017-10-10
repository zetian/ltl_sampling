# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hcost)

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(GRAPH_LIB_SRC
	spot_hoa_interpreter.cpp
	cbba_Agent_sampling.cpp
	ltl_formula_sampling.cpp
)
add_library(transsys STATIC ${GRAPH_LIB_SRC})
target_link_libraries(transsys spot bddx)

# Add executables
#add_executable(run_h2c h2c.cpp)
#target_link_libraries(build_graph graph ${OpenCV_LIBS})
#target_link_libraries(run_h2c h2c spot bddx)
