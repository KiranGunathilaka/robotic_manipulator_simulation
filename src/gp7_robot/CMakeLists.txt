cmake_minimum_required(VERSION 3.8)
project(gp7_robot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(xacro REQUIRED)
find_package(ament_cmake_python REQUIRED)

install(
  DIRECTORY urdf meshes config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES model.config
  DESTINATION share/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  gp7_robot/scripts/pick_place_controller.py
  gp7_robot/scripts/test_kinematics.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
