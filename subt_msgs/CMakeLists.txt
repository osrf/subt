cmake_minimum_required(VERSION 2.8.3)
project(subt_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  std_msgs
)

add_service_files(
  FILES
    DatagramRos.srv
    Bind.srv
    SetPose.srv
    SetRate.srv
    PoseFromArtifact.srv
    Register.srv
    Unregister.srv
)

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
