cmake_minimum_required(VERSION 3.14)
project(offline_nvtl_tool)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/offline_nvtl_tool.cpp
  src/map_loader.cpp
  src/ndt_interface.cpp
  src/topic_synchronizer.cpp
)

install(PROGRAMS
  scripts/plot_nvtl.py
  scripts/make_trajectory.py
  scripts/yaml_overwriter.py
  scripts/offline_nvtl_tool.py
  scripts/validate.py
  DESTINATION lib/${PROJECT_NAME})

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
