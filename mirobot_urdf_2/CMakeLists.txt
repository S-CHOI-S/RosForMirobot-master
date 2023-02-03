cmake_minimum_required(VERSION 2.8.3)
project(mirobot_urdf_2)

find_package(catkin REQUIRED
	gazebo_ros
	joint_state_publisher
	robot_state_publisher
	rviz
	serial
        roscpp
        rospy
        std_msgs
        sensor_msgs
	controller_manager
	tf
)

find_package(gazebo REQUIRED
## ADD!
  roscpp
  std_msgs
  message_generation
)

## ADD!
 add_message_files(
   FILES
   MsgTutorial.msg
 )

## ADD!
 generate_messages(
   DEPENDENCIES
   std_msgs
 )

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${SDFormat_INCLUDE_DIRS})

catkin_package(
	DEPENDS
  gazebo_ros
	joint_state_publisher
	robot_state_publisher
	rviz
	serial
  CATKIN_DEPENDS
  INCLUDE_DIRS
  LIBRARIES
  roscpp
  rospy
  std_msgs
  sensor_msgs
  controller_manager
)

find_package(roslaunch)

add_executable(mirobot_write_node src/mirobot_write_node.cpp)
target_link_libraries(mirobot_write_node ${catkin_LIBRARIES})

## ADD!
add_executable(topic_pub src/topic_pub.cpp)
target_link_libraries(topic_pub ${catkin_LIBRARIES})
##add_dependencies(topic_pub oroca_ros_tutorials_generate_messages_cpp)

add_executable(topic_sub src/topic_sub.cpp)
target_link_libraries(topic_sub ${catkin_LIBRARIES})
##add_dependencies(topic_sub oroca_ros_tutorials_generate_messages_cpp)

add_executable(topic_pub2 src/topic_pub2.cpp)
target_link_libraries(topic_pub2 ${catkin_LIBRARIES})
##add_dependencies(topic_pub2 oroca_ros_tutorials_generate_messages_cpp)

add_executable(topic_sub2 src/topic_sub2.cpp)
target_link_libraries(topic_sub2 ${catkin_LIBRARIES})
##add_dependencies(topic_sub oroca_ros_tutorials_generate_messages_cpp)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)

