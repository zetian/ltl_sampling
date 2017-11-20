# Include path
find_package(GUROBI REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${GUROBI_INCLUDE_DIRS})



# Add libraries
set(POLYOPT_LIB_SRC
    src/polyopt_math.cpp
    src/gurobi_solver/gurobi_utils.cpp
    src/gurobi_solver/gurobi_polyopt.cpp
)
add_library(polyopt STATIC ${POLYOPT_LIB_SRC})
target_link_libraries(polyopt common ${GUROBI_LIBRARIES})
target_include_directories(polyopt PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/inc>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

# Add executables
add_subdirectory(tests)