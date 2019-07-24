cmake_minimum_required(VERSION 2.8.3)
project(subt_rotors_controller)

add_definitions(-std=c++11)

find_package(Eigen3 REQUIRED)

catkin_package(
  CATKIN_DEPENDS rotors_control
)

find_package(catkin REQUIRED COMPONENTS
  rotors_control
  cmake_modules
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(roll_pitch_yawrate_thrust_controller_node
  src/roll_pitch_yawrate_thrust_controller_node.cpp)
add_dependencies(roll_pitch_yawrate_thrust_controller_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(roll_pitch_yawrate_thrust_controller_node
  ${catkin_LIBRARIES})

install(TARGETS roll_pitch_yawrate_thrust_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
