# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(SAMPLING_LIB_SRC
	sample_node.cpp
)
add_library(sampling STATIC ${SAMPLING_LIB_SRC})
target_link_libraries(sampling)
#add_library(transsys STATIC ${GRAPH_LIB_SRC})
#target_link_libraries(transsys spot bddx)

# Add executables
#add_executable(run_h2c h2c.cpp)
#target_link_libraries(build_graph graph ${OpenCV_LIBS})
#target_link_libraries(run_h2c h2c spot bddx)

