cmake_minimum_required(VERSION 3.0.2)
project(corridor_raiser)

add_compile_options(-std=c++14)
set(EIGEN3_INCLUDE_DIR /usr/include/eigen3)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    message_generation std_msgs geometry_msgs)
find_package (GAZEBO REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES corridor_raiser
#  CATKIN_DEPENDS other_catkin_pkg
  DEPENDS gazebo_ros
)

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${catkin_INCLUDE_DIRS}
    ../sparcslib/include
)

add_library (corridor_raiser_physics SHARED src/corridor_raiser_physics_plugin.cpp)
target_link_libraries(corridor_raiser_physics ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(corridor_raiser_physics ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library (corridor_raiser_visual SHARED src/corridor_raiser_visual_plugin.cpp)
target_link_libraries(corridor_raiser_visual ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(corridor_raiser_visual ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


add_executable (corridor_raiser_node src/corridor_raiser_node.cpp src/corridor_raiser.cpp)
target_link_libraries (corridor_raiser_node ${catkin_LIBRARIES} sparcsnode systems_lib)
add_dependencies(corridor_raiser_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
