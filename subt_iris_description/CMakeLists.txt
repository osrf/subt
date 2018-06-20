cmake_minimum_required(VERSION 3.5.1)
project(subt_iris_description)


find_package(catkin REQUIRED
  COMPONENTS
    rotors_gazebo
)

catkin_package(
  # CATKIN_DEPENDS message_runtime
)

########################
## Message generation ##
########################

# No messages to build

###########
## Build ##
###########

# No binarys to build

###########
## Tests ##
###########


#############
## Install ##
#############

# Install all the launch files
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install all the urdf file
install(DIRECTORY urdf
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
