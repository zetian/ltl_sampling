# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
#set(GRAPH_LIB_SRC
	#graph.cpp
	#graph_builder.cpp
#)
#add_library(graph STATIC ${GRAPH_LIB_SRC})
#target_link_libraries(graph qtree)

# Add executables
#add_executable(test_template test_template.cpp)
#target_link_libraries(test_template graph)
