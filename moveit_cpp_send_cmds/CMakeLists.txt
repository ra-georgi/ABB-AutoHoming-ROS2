cmake_minimum_required(VERSION 3.8)
project(moveit_cpp_send_cmds)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(moveit_cpp_send_cmds src/moveit_cpp_send_cmds.cpp)

target_include_directories(moveit_cpp_send_cmds PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(moveit_cpp_send_cmds PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  moveit_cpp_send_cmds
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS moveit_cpp_send_cmds
  DESTINATION lib/${PROJECT_NAME})

# New node
add_executable(orion_cmd src/orion_cmd.cpp)

target_include_directories(orion_cmd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(orion_cmd PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  orion_cmd
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS orion_cmd
  DESTINATION lib/${PROJECT_NAME})

# New node
add_executable(yumi_cmd_right src/yumi_cmd_right.cpp)

target_include_directories(yumi_cmd_right PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(yumi_cmd_right PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  yumi_cmd_right
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS yumi_cmd_right
  DESTINATION lib/${PROJECT_NAME})
  
# New node
add_executable(yumi_cmd src/yumi_cmd.cpp)

target_include_directories(yumi_cmd PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(yumi_cmd PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
ament_target_dependencies(
  yumi_cmd
  "moveit_ros_planning_interface"
  "rclcpp"
)

install(TARGETS yumi_cmd
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY meshes
        DESTINATION share/${PROJECT_NAME})

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

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
