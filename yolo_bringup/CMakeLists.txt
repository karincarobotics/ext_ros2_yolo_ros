cmake_minimum_required(VERSION 3.8)
project(yolo_bringup)

# Suppress warning for unused CMAKE_EXPORT_COMPILE_COMMANDS
# just declare it so it'll be used and CMAKE won't complain
set(ignoreMe "${CMAKE_EXPORT_COMPILE_COMMANDS}")


# find dependencies
find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  models
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
