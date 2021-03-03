cmake_minimum_required(VERSION 2.8.3)
project(ign_migration_scripts)

find_package(catkin REQUIRED)

catkin_package()

catkin_add_env_hooks(29.ign_migration_scripts SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
