# Include path
#include_directories(${CMAKE_SOURCE_DIR}/inc)
include_directories(${CMAKE_SOURCE_DIR}/src)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Dependency libraries
#find_package(OpenCV REQUIRED)

## Find LCM
find_package(LCM REQUIRED)

if(LCM_NOT_FOUND)
  message(STATUS "LCM is not found. Inter-process communication depends on LCM.")
endif(LCM_NOT_FOUND)

# Include files for LCM
set(LCM_GENERATED_HEADER_PATH ${CMAKE_BINARY_DIR}/include)
include_directories(${LCM_GENERATED_HEADER_PATH})


# Add executables


add_executable(test_sample_v2 test_sample_v2.cpp)
target_link_libraries(test_sample_v2 transsys trajectory sampling utilities ${LCM_LIBRARIES})

# add_executable(test_3d test_3d.cpp)
# target_link_libraries(test_3d transsys trajectory sampling utilities polyopt ${GUROBI_LIBRARIES} ${LCM_LIBRARIES})

# add_executable(test_quad test_quad.cpp)
# target_link_libraries(test_quad transsys trajectory sampling polyopt ${GUROBI_LIBRARIES} ${LCM_LIBRARIES})

# add_executable(test_quad_2 test_quad_2.cpp)
# target_link_libraries(test_quad_2 transsys trajectory sampling polyopt ${GUROBI_LIBRARIES} ${LCM_LIBRARIES})

add_executable(test_sample_dubins test_sample_dubins.cpp)
target_link_libraries(test_sample_dubins transsys trajectory sampling utilities ${LCM_LIBRARIES})


#add_executable(test_sample_dubins_v2 test_sample_dubins_v2.cpp)
#target_link_libraries(test_sample_dubins_v2 transsys trajectory sampling ${LCM_LIBRARIES})

#add_executable(test_cbba test_cbba.cpp)
#target_link_libraries(test_cbba transsys trajectory sampling ${LCM_LIBRARIES})

#add_executable(test_cbba_v2 test_cbba_v2.cpp)
#target_link_libraries(test_cbba_v2 transsys trajectory sampling ${LCM_LIBRARIES})

#add_executable(test_multi test_multi.cpp)
#target_link_libraries(test_multi transsys trajectory multi_sampling sampling ${LCM_LIBRARIES})

add_executable(test_multi_dubins test_multi_dubins.cpp)
target_link_libraries(test_multi_dubins transsys trajectory multi_sampling sampling utilities ${LCM_LIBRARIES})

#add_executable(test_multi_dubins_v2 test_multi_dubins_v2.cpp)
#target_link_libraries(test_multi_dubins_v2 transsys trajectory multi_sampling sampling ${LCM_LIBRARIES})

#add_executable(cbba_example cbba_example.cpp)
#target_link_libraries(cbba_example transsys)