cmake_minimum_required(VERSION 3.20)
project(mcl_localization)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
##########################################
# Generate Service
##########################################
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Particles.srv"
)
rosidl_target_interfaces(mcl_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
rosidl_target_interfaces(experiment_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)
include_directories(include)

add_executable(mcl_node
  src/mcl_node.cpp
  src/mcl.cpp
  src/landmark_manager.cpp

)

ament_target_dependencies(mcl_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
)

install(TARGETS
  mcl_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  include/
  DESTINATION include
)
install(FILES
  landmarks.csv
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})



  add_executable(experiment_node
  src/experiment_node.cpp
  src/mcl.cpp
  src/landmark_manager.cpp
)

ament_target_dependencies(experiment_node
  rclcpp
  nav_msgs
  sensor_msgs
  geometry_msgs
  
)
install(TARGETS experiment_node DESTINATION lib/${PROJECT_NAME})
# IMPORTANT: Bind service interfaces
# rosidl_target_interfaces(experiment_node
#   ${PROJECT_NAME} "rosidl_typesupport_cpp"
# )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
