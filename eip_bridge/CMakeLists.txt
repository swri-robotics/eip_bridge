cmake_minimum_required(VERSION 2.8.3)
project(eip_bridge)

find_package(catkin REQUIRED COMPONENTS
  rospy
  eip_msgs
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    eip_msgs
    rospy
)

#############
## Install ##
#############

catkin_install_python(
  PROGRAMS
    src/eip_bridge/eip_bridge_node.py
    src/eip_bridge/eip_bridge_simulator_node.py
    src/eip_bridge/eip_bridge.py
    src/eip_bridge/eip_functions.py
    src/eip_bridge/eip_publisher.py
    src/eip_bridge/eip_service.py
    src/eip_bridge/eip_subscriber.py
    src/eip_bridge/plc_simulator.py
  DESTINATION
    ${CATKIN_PACKAGE_BIN_DESTINATION}
)

foreach(dir example launch)
  install(DIRECTORY ${dir}/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()
