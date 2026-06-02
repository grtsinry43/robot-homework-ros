cmake_minimum_required(VERSION 3.8)
project(panda_sim_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  models
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
