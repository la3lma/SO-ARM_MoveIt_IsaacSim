cmake_minimum_required(VERSION 3.10.2)

project(so_arm_urdf)

find_package(ament_cmake REQUIRED)
find_package(urdf REQUIRED)

install(
	DIRECTORY
		meshes
		urdf
		config
		launch
	DESTINATION
		share/${PROJECT_NAME}
)

ament_package()

