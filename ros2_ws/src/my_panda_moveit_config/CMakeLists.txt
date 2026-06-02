cmake_minimum_required(VERSION 3.22)
project(my_panda_moveit_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(
  FILES config/moveit_setup_assistant.yaml
  DESTINATION share/${PROJECT_NAME}
  RENAME .setup_assistant)
