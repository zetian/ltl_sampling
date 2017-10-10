# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
find_package(OpenCV REQUIRED)

## Find LCM
find_package(LCM REQUIRED)

if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)

# Include files for LCM
set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
include_directories(${LCM_GENERATED_HEADER_PATH})


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

#add_executable(test_sample test_sample.cpp)
#target_link_libraries(test_sample transsys trajectory sampling ${LCM_LIBRARIES})

add_executable(test_sample_v2 test_sample_v2.cpp)
target_link_libraries(test_sample_v2 transsys trajectory sampling ${LCM_LIBRARIES})

add_executable(test_sample_dubins test_sample_dubins.cpp)
target_link_libraries(test_sample_dubins transsys trajectory sampling ${LCM_LIBRARIES})

add_executable(test_sample_dubins_v2 test_sample_dubins_v2.cpp)
target_link_libraries(test_sample_dubins_v2 transsys trajectory sampling ${LCM_LIBRARIES})

add_executable(test_cbba test_cbba.cpp)
target_link_libraries(test_cbba transsys trajectory sampling ${LCM_LIBRARIES})

add_executable(test_cbba_v2 test_cbba_v2.cpp)
target_link_libraries(test_cbba_v2 transsys trajectory sampling ${LCM_LIBRARIES})

add_executable(cbba_example cbba_example.cpp)
target_link_libraries(cbba_example transsys)