cmake_minimum_required(VERSION 3.5)
project(robotiq_2f_140_gripper_visualization)

find_package(ament_cmake REQUIRED)

install(
        DIRECTORY config meshes launch urdf
        DESTINATION share/${PROJECT_NAME}
)

ament_package()