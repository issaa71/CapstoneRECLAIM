cmake_minimum_required(VERSION 3.8)
project(reclaim_arm_moveit_config)

find_package(ament_cmake REQUIRED)

# Install config files
install(DIRECTORY config launch srdf meshes
  DESTINATION share/${PROJECT_NAME}
)

# Install scripts
install(PROGRAMS
  scripts/load_planning_scene.py
  scripts/verify_camera_transform.py
  scripts/pick_and_place.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install URDF from reclaim_control (symlink or copy handled by launch file)

ament_package()
