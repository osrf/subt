set (files 
  red_grid.png
  green_grid.png
  blue_grid.png
  gate.png
)

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/gazebo_models/environments/${MODEL_NAME}/materials/textures)
