cmake_minimum_required(VERSION 3.8)
project(semantic_fleet)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Generate custom messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/DetectedObjects.msg"
  "msg/SemanticMap.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python module directory
install(DIRECTORY semantic_fleet/
  DESTINATION lib/python3.10/site-packages/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

# Install Python executables (without .py extension)
install(PROGRAMS
  semantic_fleet/yolo_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_detector
)

install(PROGRAMS
  semantic_fleet/object_localizer.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME object_localizer
)

install(PROGRAMS
  semantic_fleet/semantic_mapper.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME semantic_mapper
)

# Install launch files
install(DIRECTORY
  launch
  config
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}/
)

# Install models directory
install(DIRECTORY models/
  DESTINATION share/${PROJECT_NAME}/models/
)

ament_package()

