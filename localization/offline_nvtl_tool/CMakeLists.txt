cmake_minimum_required(VERSION 3.14)
project(offline_nvtl_tool)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/offline_nvtl_tool.cpp
  src/ndt_interface.cpp
)

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
  scripts
)
