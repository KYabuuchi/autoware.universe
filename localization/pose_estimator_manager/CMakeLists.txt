cmake_minimum_required(VERSION 3.14)
project(pose_estimator_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_executable(${PROJECT_NAME}
  src/pose_estimator_manager_node.cpp
  src/pose_estimator_manager_core.cpp
)
ament_auto_add_executable(ndt_sub_manager
  src/ndt_sub_manager.cpp
)
ament_auto_add_executable(pcdless_sub_manager
  src/pcdless_sub_manager.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include)

ament_auto_package(INSTALL_TO_SHARE
  launch
)
