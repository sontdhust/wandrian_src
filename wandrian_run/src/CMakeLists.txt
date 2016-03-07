##############################################################################
# Sources
##############################################################################

file(GLOB_RECURSE SOURCES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} *.cpp)

##############################################################################
# Targets
##############################################################################

add_executable(algorithm ${SOURCES})
add_dependencies(algorithm kobuki_msgs_gencpp)
target_link_libraries(algorithm ${catkin_LIBRARIES})

install(TARGETS algorithm
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
