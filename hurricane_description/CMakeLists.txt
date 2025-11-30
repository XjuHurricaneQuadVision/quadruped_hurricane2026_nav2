cmake_minimum_required(VERSION 3.8)
project(hurricane_description)

find_package(ament_cmake REQUIRED)

install(
		DIRECTORY meshes xacro launch config urdf model
		DESTINATION share/${PROJECT_NAME}/
)

ament_package()