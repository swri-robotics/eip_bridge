cmake_minimum_required(VERSION 2.8.3)
project(eip_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
)

################################################
## Declare ROS messages, services and actions ##
################################################
## Generate services in the 'srv' folder
add_service_files(
    FILES
    ReadTagArrayInt8.srv
    ReadTagArrayInt16.srv
    ReadTagArrayInt32.srv
    ReadTagArrayInt64.srv
    ReadTagArrayFloat32.srv
    ReadTagBool.srv
    ReadTagInt8.srv
    ReadTagInt16.srv
    ReadTagInt32.srv
    ReadTagInt64.srv
    ReadTagString.srv
    ReadTagFloat32.srv
    WriteTagArrayInt8.srv
    WriteTagArrayInt16.srv
    WriteTagArrayInt32.srv
    WriteTagArrayInt64.srv
    WriteTagArrayFloat32.srv
    WriteTagBool.srv
    WriteTagInt8.srv
    WriteTagInt16.srv
    WriteTagInt32.srv
    WriteTagInt64.srv
    WriteTagFloat32.srv
    WriteTagString.srv
    SetMode.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(CATKIN_DEPENDS message_runtime)
