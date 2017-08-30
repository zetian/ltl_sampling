# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/hcost)

# Dependency libraries
#find_package(OpenCV REQUIRED)

# Add libraries
set(GRAPH_LIB_SRC
	#graph.cpp
	#defn_graph.cpp
	#defn_history.cpp
	#defn_vertex.cpp
	#graph_h_search.cpp
	#graph_search.cpp
	spot_hoa_interpreter.cpp
	buchi_automaton.cpp
	graph_lifter.cpp
	product_automaton.cpp
	#hcost_tile_library_init.cpp
	hcost/defn_tile.cpp
	hcost/defn_hlevel.cpp
	hcost/defn_tile_block.cpp	
	hcost/hcost_interface.cpp
)
add_library(transsys STATIC ${GRAPH_LIB_SRC})
target_link_libraries(transsys spot bddx)

# Add executables
#add_executable(run_h2c h2c.cpp)
#target_link_libraries(build_graph graph ${OpenCV_LIBS})
#target_link_libraries(run_h2c h2c spot bddx)
