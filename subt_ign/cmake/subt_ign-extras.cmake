find_package(ignition-transport8 REQUIRED)

list(APPEND catkin_INCLUDE_DIRS ${ignition-transport8_INCLUDE_DIRS})
list(APPEND catkin_LIBRARIES ${ignition-transport8_LIBRARIES} ${Protobuf_LIBRARIES})
