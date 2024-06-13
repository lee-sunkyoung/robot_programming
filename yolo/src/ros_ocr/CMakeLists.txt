cmake_minimum_required(VERSION 3.5)
project(ros_ocr)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)

install(
    DIRECTORY ros_ocr
    DESTINATION lib/python3.8/site-packages/${PROJECT_NAME}
)

ament_package()