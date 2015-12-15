##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################

add_executable(run_algorithm ${SOURCES})
add_dependencies(run_algorithm kobuki_msgs_gencpp)
target_link_libraries(run_algorithm ${catkin_LIBRARIES})

install(TARGETS run_algorithm
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
