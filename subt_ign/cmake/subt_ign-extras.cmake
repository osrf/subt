find_package(ignition-transport7 REQUIRED)

list(APPEND catkin_INCLUDE_DIRS ${ignition-transport7_INCLUDE_DIRS})
list(APPEND catkin_LIBRARIES ${ignition-transport7_LIBRARIES} ${Protobuf_LIBRARIES})
