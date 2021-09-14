cmake_minimum_required(VERSION 2.8.3)

project(kuka_d435)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  rviz
  moveit_core
	moveit_ros_planning
	moveit_ros_planning_interface
  moveit_visual_tools
  std_msgs
  tf
  urdf
  xacro
)
find_package(Eigen3 REQUIRED)
FIND_PACKAGE(yaml-cpp REQUIRED)
catkin_package()

#if (CATKIN_ENABLE_TESTING)
#  find_package(roslaunch REQUIRED)
#  roslaunch_add_file_check(test/roslaunch_kr6.xml)
#endif()

install(DIRECTORY config launch meshes urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

add_executable(my_tf_broadcaster src/tf_broadcaster.cpp)
add_executable(moveit_kuka src/moveit_kuka.cpp)
add_executable(listener src/tf_listener.cpp)
add_dependencies(my_tf_broadcaster listener moveit_kuka ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(PROGRAMS
   scripts/waypoints_generator.py
   src/image_saver.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
target_link_libraries(my_tf_broadcaster
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${EIGEN3_LIBRARIES}
 )
target_link_libraries(listener
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${EIGEN3_LIBRARIES}
 )
target_link_libraries(moveit_kuka
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
   ${EIGEN3_LIBRARIES}
   yaml-cpp
 )
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)