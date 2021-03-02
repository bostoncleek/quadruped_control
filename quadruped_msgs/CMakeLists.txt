cmake_minimum_required(VERSION 3.16.3)
project(quadruped_msgs)

find_package(catkin REQUIRED COMPONENTS
	geometry_msgs
	message_generation
	std_msgs
)

add_message_files(
  FILES
  CoMState.msg
	JointTorqueCmd.msg
)

# add_service_files(
#   FILES
# )

# add_action_files( 
#   FILES
# )

generate_messages(
  DEPENDENCIES
	geometry_msgs
  std_msgs
)

catkin_package(
 CATKIN_DEPENDS
 	geometry_msgs
 	message_runtime
	std_msgs
)

# # Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
