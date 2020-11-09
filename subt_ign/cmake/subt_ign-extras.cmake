find_package(ignition-transport9 REQUIRED)

list(APPEND catkin_INCLUDE_DIRS ${ignition-transport9_INCLUDE_DIRS})
list(APPEND catkin_LIBRARIES ${ignition-transport9_LIBRARIES} ${Protobuf_LIBRARIES})
