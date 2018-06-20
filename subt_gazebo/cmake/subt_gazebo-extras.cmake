find_package(ignition-transport4 REQUIRED)

list(APPEND catkin_INCLUDE_DIRS ${ignition-transport4_INCLUDE_DIRS})
list(APPEND catkin_LIBRARIES ${ignition-transport4_LIBRARIES} ${Protobuf_LIBRARIES})
