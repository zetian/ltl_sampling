# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add executables
add_executable(simple_graph simple_graph.cpp)

add_executable(graph_example graph_example.cpp)
target_link_libraries(graph_example vis map ${OpenCV_LIBS})

#add_executable(product_example product_example.cpp)
#target_link_libraries(product_example transsys vis map ${OpenCV_LIBS})

#add_executable(test_tile_data test_tile_data.cpp)
#target_link_libraries(test_tile_data h2c vis map ${OpenCV_LIBS})

#add_executable(calc_hcost_main calc_hcost_main.cpp)
#target_link_libraries(calc_hcost_main transsys vis map ${OpenCV_LIBS})

#add_executable(cbta_example cbta_example.cpp)
#target_link_libraries(cbta_example transsys vis map ${OpenCV_LIBS})
