# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

# Add executables
#add_executable(test_buchi test_buchi.cpp)
#target_link_libraries(test_buchi transsys vis map ${OpenCV_LIBS})

# add_executable(test_product test_product.cpp)
# target_link_libraries(test_product transsys vis map ${OpenCV_LIBS})

# add_executable(test_memory test_memory.cpp)
# target_link_libraries(test_memory transsys vis map ${OpenCV_LIBS})

#add_executable(test_product test_product.cpp)
#target_link_libraries(test_product transsys vis map ${OpenCV_LIBS})

#add_executable(test_tile_data test_tile_data.cpp)
#target_link_libraries(test_tile_data transsys vis map ${OpenCV_LIBS})

#add_executable(test_lifted_transition test_lifted_transition.cpp)
#target_link_libraries(test_lifted_transition transsys vis map ${OpenCV_LIBS})

add_executable(test_dubins test_dubins.cpp)
target_link_libraries(test_dubins trajectory)

