# Include path
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# Add executables
#add_executable(qp_cpp tests/qp_c++.cpp)
#target_link_libraries(qp_cpp ${GUROBI_LIBRARIES})

#add_executable(test_utils tests/test_utils.cpp)
#target_link_libraries(test_utils polyopt ${GUROBI_LIBRARIES})

#add_executable(test_qp tests/test_qp.cpp)
#target_link_libraries(test_qp polyopt vis ${GUROBI_LIBRARIES})

#add_executable(test_qp2 tests/test_qp2.cpp)
#target_link_libraries(test_qp2 polyopt ${GUROBI_LIBRARIES})

#add_executable(test_qp3 tests/test_qp3.cpp)
#target_link_libraries(test_qp3 polyopt ${GUROBI_LIBRARIES})

add_executable(test_opt test_opt.cpp)
target_link_libraries(test_opt polyopt)

#add_executable(test_corridor tests/test_corridor.cpp)
#target_link_libraries(test_corridor polyopt ${GUROBI_LIBRARIES} ${LCM_LIBRARIES})

#add_executable(test_cgal_qp test_cgal_qp.cpp)
#target_link_libraries(test_cgal_qp polyopt ${CGAL_LIBRARIES} ${GMP_LIBRARIES})
