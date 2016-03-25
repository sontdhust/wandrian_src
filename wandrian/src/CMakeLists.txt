##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################

add_executable(run ${SOURCES})
add_dependencies(run kobuki_msgs_gencpp)
target_link_libraries(run ${catkin_LIBRARIES})

install(TARGETS run
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
