cmake_minimum_required(VERSION 3.10.0)
project(bosdyn_spot)

set(CMAKE_CXX_STANDARD 17)

set(IGN_GAZEBO_VER 4)
find_package(ignition-gazebo${IGN_GAZEBO_VER} REQUIRED)

set(DEPS control_toolbox dynamic_reconfigure ros_ign_bridge nodelet roscpp)
set(MSG_DEPS std_msgs geometry_msgs)

find_package(catkin REQUIRED COMPONENTS ${DEPS} ${MSG_DEPS} message_generation)

add_message_files(FILES Contact.msg ContactsStamped.msg LogicalContact.msg)
generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS ${DEPS} ${MSG_DEPS} message_runtime)

add_library(multi-joint-command-system src/multi_joint_command.cpp)
target_link_libraries(multi-joint-command-system
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::core
)

add_library(foot_contact_bridge src/foot_contact_bridge.cpp)
target_include_directories(foot_contact_bridge PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(foot_contact_bridge
  PUBLIC ignition-gazebo${IGN_GAZEBO_VER}::core ${catkin_LIBRARIES}
)
add_dependencies(foot_contact_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(logical_contact_bridge src/logical_contact_bridge.cpp)
target_include_directories(logical_contact_bridge PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(logical_contact_bridge
  PUBLIC ignition-gazebo${IGN_GAZEBO_VER}::core ${catkin_LIBRARIES}
)
add_dependencies(logical_contact_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(logical-contact-system src/logical_contact_system.cpp)
target_link_libraries(logical-contact-system
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::core
)

add_library(pid_control_bridge src/pid_control_bridge.cpp)
target_include_directories(pid_control_bridge PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(pid_control_bridge
  PUBLIC ignition-gazebo${IGN_GAZEBO_VER}::core ${catkin_LIBRARIES}
)
add_dependencies(pid_control_bridge ${catkin_EXPORTED_TARGETS})

add_library(joint_trajectory_bridge src/joint_trajectory_bridge.cpp)
target_include_directories(joint_trajectory_bridge PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(joint_trajectory_bridge
  PUBLIC ignition-gazebo${IGN_GAZEBO_VER}::core ${catkin_LIBRARIES}
)
add_dependencies(joint_trajectory_bridge ${catkin_EXPORTED_TARGETS})

add_library(configurable-joint-position-controller-system src/configurable_joint_position_controller.cpp)
target_link_libraries(configurable-joint-position-controller-system
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::core
)

add_library(mono-camera-system src/mono_camera_system.cpp)
target_link_libraries(mono-camera-system
  PRIVATE ignition-gazebo${IGN_GAZEBO_VER}::core
)

install(TARGETS
    configurable-joint-position-controller-system
    joint_trajectory_bridge
    multi-joint-command-system
    pid_control_bridge
    foot_contact_bridge
    logical-contact-system
    logical_contact_bridge
    mono-camera-system
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(PROGRAMS scripts/clean_sdf scripts/high_precision_constants.py scripts/print_robot_urdf scripts/yaml_to_xacro_args
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts)

install(DIRECTORY config launch meshes urdf worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(FILES model.sdf model.config nodelets.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

catkin_install_python(PROGRAMS nodes/pid_multituner
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})