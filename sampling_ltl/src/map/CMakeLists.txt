# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add libraries
set(MAP_LIB_SRC
    #map_manager.cpp
    image_utils.cpp
    map_utils.cpp
    sgrid_builder.cpp
    graph_builder.cpp
    square_grid.cpp
)
add_library(map STATIC ${MAP_LIB_SRC})
target_link_libraries(map ${OpenCV_LIBS})

# Add executables
#add_executable(test_graph test_graph.cpp)
#target_link_libraries(test_graph h2c graph_vis ${OpenCV_LIBS})
