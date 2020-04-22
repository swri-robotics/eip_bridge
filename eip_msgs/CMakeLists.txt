cmake_minimum_required(VERSION 2.8.3)
project(eip_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
)

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

generate_messages()

catkin_package(CATKIN_DEPENDS message_runtime)
