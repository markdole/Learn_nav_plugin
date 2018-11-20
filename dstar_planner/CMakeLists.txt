cmake_minimum_required(VERSION 2.8.3)
project(dstar_planner)

find_package(catkin REQUIRED
        COMPONENTS
        base_local_planner
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf2
        tf2_geometry_msgs
        tf2_ros
        )

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)
add_definitions(${EIGEN3_DEFINITIONS})

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES dstar_planner
        CATKIN_DEPENDS
        base_local_planner
        costmap_2d
        nav_core
        pluginlib
        roscpp
        tf2
        tf2_ros
)

add_library(dstar_planner src/dstar_planner.cpp include/dstar_planner/dstar_planner.h)
#  the main cpp

add_dependencies(dstar_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(dstar_planner
        ${catkin_LIBRARIES}
        )
        
################################################################################
# Install
################################################################################
install(TARGETS dstar_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

install(FILES dstar_planner_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )


