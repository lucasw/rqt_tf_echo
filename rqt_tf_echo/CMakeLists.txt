cmake_minimum_required(VERSION 3.0.2)
project(rqt_tf_echo)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roslint
  rospy
  rqt_gui_py
  std_msgs
)

roslint_python()

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS rospy rqt_gui_py std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(PROGRAMS
  scripts/${PROJECT_NAME}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES
  plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
