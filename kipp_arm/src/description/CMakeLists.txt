# Install launch directory
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config, rviz, urdf and meshes directory
install(DIRECTORY
  config
  rviz
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}/
)
