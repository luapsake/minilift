cmake_minimum_required(VERSION 3.5)
project(minilift_navigation)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch rviz config maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
